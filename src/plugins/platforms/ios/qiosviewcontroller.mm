// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only

#include "qiosglobal.h"
#import "qiosviewcontroller.h"

#include <QtCore/qscopedvaluerollback.h>
#include <QtCore/private/qcore_mac_p.h>
#include <QtGui/private/qapplekeymapper_p.h>

#include <QtGui/QGuiApplication>
#include <QtGui/QWindow>
#include <QtGui/QScreen>

#include <QtGui/private/qwindow_p.h>
#include <QtGui/private/qguiapplication_p.h>

#include "qiosintegration.h"
#include "qiosscreen.h"
#include "qiosglobal.h"
#include "qioswindow.h"
#include "quiview.h"

#include <QtCore/qpointer.h>

// -------------------------------------------------------------------------

@interface QIOSViewController ()
@property (nonatomic, assign) QPointer<QT_PREPEND_NAMESPACE(QIOSScreen)> platformScreen;
@property (nonatomic, assign) BOOL changingOrientation;
@property (nonatomic, assign) UIDeviceOrientation preLockedOrienation;
- (void)updateToOrientation:(UIInterfaceOrientation)uiOrientation deviceOrientation:(UIDeviceOrientation)deviceOrientation;
@end

// -------------------------------------------------------------------------

@interface QIOSDesktopManagerView : UIView
@end

@implementation QIOSDesktopManagerView

- (instancetype)init
{
    if (!(self = [super init]))
        return nil;

    if (qEnvironmentVariableIntValue("QT_IOS_DEBUG_WINDOW_MANAGEMENT")) {
        static UIImage *gridPattern = nil;
        static dispatch_once_t onceToken;
        dispatch_once(&onceToken, ^{
            CGFloat dimension = 100.f;

            UIGraphicsBeginImageContextWithOptions(CGSizeMake(dimension, dimension), YES, 0.0f);
            CGContextRef context = UIGraphicsGetCurrentContext();

            CGContextTranslateCTM(context, -0.5, -0.5);

            #define gridColorWithBrightness(br) \
                [UIColor colorWithHue:0.6 saturation:0.0 brightness:br alpha:1.0].CGColor

            CGContextSetFillColorWithColor(context, gridColorWithBrightness(0.05));
            CGContextFillRect(context, CGRectMake(0, 0, dimension, dimension));

            CGFloat gridLines[][2] = { { 10, 0.1 }, { 20, 0.2 }, { 100, 0.3 } };
            for (size_t l = 0; l < sizeof(gridLines) / sizeof(gridLines[0]); ++l) {
                CGFloat step = gridLines[l][0];
                for (int c = step; c <= dimension; c += step) {
                    CGContextMoveToPoint(context, c, 0);
                    CGContextAddLineToPoint(context, c, dimension);
                    CGContextMoveToPoint(context, 0, c);
                    CGContextAddLineToPoint(context, dimension, c);
                }

                CGFloat brightness = gridLines[l][1];
                CGContextSetStrokeColorWithColor(context, gridColorWithBrightness(brightness));
                CGContextStrokePath(context);
            }

            gridPattern = UIGraphicsGetImageFromCurrentImageContext();
            UIGraphicsEndImageContext();

            [gridPattern retain];
        });

        self.backgroundColor = [UIColor colorWithPatternImage:gridPattern];
    }

    return self;
}

- (void)didAddSubview:(UIView *)subview
{
    Q_UNUSED(subview);

    QT_PREPEND_NAMESPACE(QIOSScreen) *screen = self.qtViewController.platformScreen;

    // The 'window' property of our view is not valid until the window
    // has been shown, so we have to access it through the QIOSScreen.
    UIWindow *uiWindow = screen->uiWindow();

    if (uiWindow.hidden) {
        // Associate UIWindow to screen and show it the first time a QWindow
        // is mapped to the screen. For external screens this means disabling
        // mirroring mode and presenting alternate content on the screen.
        uiWindow.screen = screen->uiScreen();
        uiWindow.hidden = NO;
    }
}

- (void)willRemoveSubview:(UIView *)subview
{
    Q_UNUSED(subview);

    UIWindow *uiWindow = self.window;
    // uiWindow can be null when closing from the ios "app manager" and the app is
    // showing a native window like UIDocumentBrowserViewController
    if (!uiWindow)
        return;

    if (uiWindow.screen != [UIScreen mainScreen] && self.subviews.count == 1) {
        // We're about to remove the last view of an external screen, so go back
        // to mirror mode, but defer it until after the view has been removed,
        // to ensure that we don't try to layout the view that's being removed.
        dispatch_async(dispatch_get_main_queue(), ^{
            uiWindow.hidden = YES;
            uiWindow.screen = [UIScreen mainScreen];
        });
    }
}

- (void)layoutSubviews
{
    if (QGuiApplication::applicationState() == Qt::ApplicationSuspended) {
        // Despite the OpenGL ES Programming Guide telling us to avoid all
        // use of OpenGL while in the background, iOS will perform its view
        // snapshotting for the app switcher after the application has been
        // backgrounded; once for each orientation. Presumably the expectation
        // is that no rendering needs to be done to provide an alternate
        // orientation snapshot, just relayouting of views. But in our case,
        // or any non-stretchable content case such as a OpenGL based game,
        // this is not true. Instead of continuing layout, which will send
        // potentially expensive geometry changes (with isExposed false,
        // since we're in the background), we short-circuit the snapshotting
        // here. iOS will still use the latest rendered frame to create the
        // application switcher thumbnail, but it will be based on the last
        // active orientation of the application.
        QIOSScreen *screen = self.qtViewController.platformScreen;
        qCDebug(lcQpaWindow) << "ignoring layout of subviews while suspended,"
            << "likely system snapshot of" << screen->screen()->primaryOrientation();
        return;
    }

    for (int i = int(self.subviews.count) - 1; i >= 0; --i) {
        UIView *view = static_cast<UIView *>([self.subviews objectAtIndex:i]);
        if (![view isKindOfClass:[QUIView class]])
            continue;

        [self layoutView: static_cast<QUIView *>(view)];
    }
}

- (void)layoutView:(QUIView *)view
{
    QWindow *window = view.qwindow;

    // Return early if the QIOSWindow is still constructing, as we'll
    // take care of setting the correct window state in the constructor.
    if (!window->handle())
        return;

    // Re-apply window states to update geometry
    if (window->windowStates() & (Qt::WindowFullScreen | Qt::WindowMaximized))
        window->handle()->setWindowState(window->windowStates());
}

// Even if the root view controller has both wantsFullScreenLayout and
// extendedLayoutIncludesOpaqueBars enabled, iOS will still push the root
// view down 20 pixels (and shrink the view accordingly) when the in-call
// statusbar is active (instead of updating the topLayoutGuide). Since
// we treat the root view controller as our screen, we want to reflect
// the in-call statusbar as a change in available geometry, not in screen
// geometry. To simplify the screen geometry mapping code we reset the
// view modifications that iOS does and take the statusbar height
// explicitly into account in QIOSScreen::updateProperties().

- (void)setFrame:(CGRect)newFrame
{
    Q_UNUSED(newFrame);
    Q_ASSERT(!self.window || self.window.rootViewController.view == self);

    // When presenting view controllers our view may be temporarily reparented into a UITransitionView
    // instead of the UIWindow, and the UITransitionView may have a transform set, so we need to do a
    // mapping even if we still expect to always be the root view-controller.
    CGRect transformedWindowBounds = [self.superview convertRect:self.window.bounds fromView:self.window];
    [super setFrame:transformedWindowBounds];
}

- (void)setBounds:(CGRect)newBounds
{
    Q_UNUSED(newBounds);
    CGRect transformedWindowBounds = [self convertRect:self.window.bounds fromView:self.window];
    [super setBounds:CGRectMake(0, 0, CGRectGetWidth(transformedWindowBounds), CGRectGetHeight(transformedWindowBounds))];
}

- (void)setCenter:(CGPoint)newCenter
{
    Q_UNUSED(newCenter);
    [super setCenter:self.window.center];
}

- (void)didMoveToWindow
{
    // The initial frame computed during startup may happen before the view has
    // a window, meaning our calculations above will be wrong. We ensure that the
    // frame is set correctly once we have a window to base our calculations on.
    [self setFrame:self.window.bounds];
}

@end

// -------------------------------------------------------------------------

@implementation QIOSViewController {
    BOOL m_updatingProperties;
    QMetaObject::Connection m_focusWindowChangeConnection;
    QMetaObject::Connection m_appStateChangedConnection;
}

#ifndef Q_OS_TVOS
@synthesize prefersStatusBarHidden;
@synthesize preferredStatusBarUpdateAnimation;
@synthesize preferredStatusBarStyle;
#endif

- (instancetype)initWithQIOSScreen:(QT_PREPEND_NAMESPACE(QIOSScreen) *)screen
{
    if (self = [self init]) {
        self.platformScreen = screen;

        self.changingOrientation = NO;
#ifndef Q_OS_TVOS
        self.lockedOrientation = UIInterfaceOrientationUnknown;
        self.preLockedOrienation = UIDevice.currentDevice.orientation;

        // Status bar may be initially hidden at startup through Info.plist
        self.prefersStatusBarHidden = infoPlistValue(@"UIStatusBarHidden", false);
        self.preferredStatusBarUpdateAnimation = UIStatusBarAnimationNone;
        self.preferredStatusBarStyle = UIStatusBarStyle(infoPlistValue(@"UIStatusBarStyle", UIStatusBarStyleDefault));
#endif

        m_focusWindowChangeConnection = QObject::connect(qApp, &QGuiApplication::focusWindowChanged, [self]() {
            [self updateProperties];
        });

        QIOSApplicationState *applicationState = &QIOSIntegration::instance()->applicationState;
        m_appStateChangedConnection = QObject::connect(applicationState, &QIOSApplicationState::applicationStateDidChange,
            [self](Qt::ApplicationState oldState, Qt::ApplicationState newState) {
                if (oldState == Qt::ApplicationSuspended && newState != Qt::ApplicationSuspended) {
                    // We may have ignored an earlier layout because the application was suspended,
                    // and we didn't want to render anything at that moment in fear of being killed
                    // due to rendering in the background, so we trigger an explicit layout when
                    // coming out of the suspended state.
                    qCDebug(lcQpaWindow) << "triggering root VC layout when coming out of suspended state";
                    [self.view setNeedsLayout];
                }
            }
        );
    }

    return self;
}

- (void)dealloc
{
    QObject::disconnect(m_focusWindowChangeConnection);
    QObject::disconnect(m_appStateChangedConnection);
    [super dealloc];
}

- (void)loadView
{
    self.view = [[[QIOSDesktopManagerView alloc] init] autorelease];
}

- (void)viewDidLoad
{
    [super viewDidLoad];

    Q_ASSERT(!qt_apple_isApplicationExtension());

#ifndef Q_OS_TVOS
    NSNotificationCenter *center = [NSNotificationCenter defaultCenter];
    [center addObserver:self selector:@selector(willChangeStatusBarFrame:)
            name:UIApplicationWillChangeStatusBarFrameNotification
            object:qt_apple_sharedApplication()];

    [center addObserver:self selector:@selector(didChangeStatusBarOrientation:)
            name:UIApplicationDidChangeStatusBarOrientationNotification
            object:qt_apple_sharedApplication()];
#endif
}

- (void)viewDidUnload
{
    [[NSNotificationCenter defaultCenter] removeObserver:self name:nil object:nil];
    [super viewDidUnload];
}

// -------------------------------------------------------------------------

- (BOOL)shouldAutorotate
{
#ifndef Q_OS_TVOS
    if (self.platformScreen && self.platformScreen->uiScreen() == [UIScreen mainScreen])
    {
        if (!self.lockedOrientation)
            return YES;
        CGSize currentSize = self.view.frame.size;
        if (UIInterfaceOrientationIsLandscape(self.lockedOrientation))
            return (currentSize.width < currentSize.height); // if width is currently less than height should allow rotation, otherwise no.
        else
            return (currentSize.height < currentSize.width); // if height is currently less than width should allow rotation, otherwise no.
    }
    else
        return NO; // no rotation on non-primary screen
#else
    return NO;
#endif
}

- (UIInterfaceOrientationMask)supportedInterfaceOrientations
{
    if (!self.lockedOrientation) // modern iPhones don't really do UpsideDown, so don't even try
        return (self.traitCollection.userInterfaceIdiom != UIUserInterfaceIdiomPhone) ? UIInterfaceOrientationMaskAll  : UIInterfaceOrientationMaskAllButUpsideDown;
    else // take advantage of the fact that the mask is just a bitshift of the the UIInterfaceOrientation values
        return (UIInterfaceOrientationMask)(1 << self.lockedOrientation);
}

- (UIInterfaceOrientation)preferredInterfaceOrientationForPresentation
{
    if (!self.lockedOrientation)
        return UIInterfaceOrientationUnknown;
    else
        return self.lockedOrientation;
}

- (void)willRotateToInterfaceOrientation:(UIInterfaceOrientation)orientation duration:(NSTimeInterval)duration
{
    self.changingOrientation = YES;

    [super willRotateToInterfaceOrientation:orientation duration:duration];
}

- (void)didRotateFromInterfaceOrientation:(UIInterfaceOrientation)orientation
{
    self.changingOrientation = NO;

    [super didRotateFromInterfaceOrientation:orientation];
}

- (void)willChangeStatusBarFrame:(NSNotification*)notification
{
    Q_UNUSED(notification);

    if (self.view.window.screen != [UIScreen mainScreen])
        return;

    // Orientation changes will already result in laying out subviews, so we don't
    // need to do anything extra for frame changes during an orientation change.
    // Technically we can receive another actual statusbar frame update during the
    // orientation change that we should react to, but to simplify the logic we
    // use a simple bool variable instead of a ignoreNextFrameChange approach.
    if (self.changingOrientation)
        return;

    // UIKit doesn't have a delegate callback for statusbar changes that's run inside the
    // animation block, like UIViewController's willAnimateRotationToInterfaceOrientation,
    // nor does it expose a constant for the duration and easing of the animation. However,
    // though poking at the various UIStatusBar methods, we can observe that the animation
    // uses the default easing curve, and runs with a duration of 0.35 seconds.
    static qreal kUIStatusBarAnimationDuration = 0.35;

    [UIView animateWithDuration:kUIStatusBarAnimationDuration animations:^{
        [self.view setNeedsLayout];
        [self.view layoutIfNeeded];
    }];
}

- (void)didChangeStatusBarOrientation:(NSNotification *)notification
{
    Q_UNUSED(notification);

    if (self.view.window.screen != [UIScreen mainScreen])
        return;

    // If the statusbar changes orientation due to auto-rotation we don't care,
    // there will be re-layout anyways. Only if the statusbar changes due to
    // reportContentOrientation, we need to update the window layout.
    if (self.changingOrientation)
        return;

    [self.view setNeedsLayout];
}

- (void)viewWillLayoutSubviews
{
    if (!QCoreApplication::instance())
        return;

//    NSLog(@"%@:viewWillLayoutSubviews", self);
    if (self.platformScreen)
        self.platformScreen->updateProperties();
}

- (void)viewWillTransitionToSize:(CGSize)size withTransitionCoordinator:(id<UIViewControllerTransitionCoordinator>)coordinator
{
    NSLog(@"viewWillTransitionToSize:%@ withTransitionCoordinator:%@", NSStringFromCGSize(size), coordinator);
    [super viewWillTransitionToSize:size withTransitionCoordinator:coordinator];
    
    [coordinator animateAlongsideTransition:^(id<UIViewControllerTransitionCoordinatorContext> context) {
    } completion:^(id<UIViewControllerTransitionCoordinatorContext> context) {
        NSLog(@"viewWillTransitionToSize:%@ - COMPLETION!", NSStringFromCGSize(size));
        if (!QCoreApplication::instance())
            return;

        if (self.platformScreen)
            self.platformScreen->updateGeometry();
    }];
}
// -------------------------------------------------------------------------

- (UIInterfaceOrientation)toUIInterfaceOrientation:(Qt::ScreenOrientation)qtOrientation
{
    UIInterfaceOrientation uiOrientation;
    switch (qtOrientation) {
    case Qt::LandscapeOrientation:
        uiOrientation = UIInterfaceOrientationLandscapeRight;
        break;
    case Qt::InvertedLandscapeOrientation:
        uiOrientation = UIInterfaceOrientationLandscapeLeft;
        break;
    case Qt::InvertedPortraitOrientation:
        uiOrientation = UIInterfaceOrientationPortraitUpsideDown;
        break;
    case Qt::PrimaryOrientation:
    case Qt::PortraitOrientation:
    default:
        uiOrientation = UIInterfaceOrientationPortrait;
        break;
    }
    return uiOrientation;
}

- (void)updateToOrientation:(UIInterfaceOrientation)uiOrientation deviceOrientation:(UIDeviceOrientation)deviceOrientation
{
    self.lockedOrientation = uiOrientation;
    
    // Detect if this iOS16 method is available
	if (@available(ios 16, macCatalyst 16.0, tvOS 16, *)) {
		[self setNeedsUpdateOfSupportedInterfaceOrientations];

		// Now check if we're in a scene, only need to do it if we somehow ended up there
		Q_ASSERT(self.view.window);
		UIWindowScene* scene =self.view.window.windowScene;
			
		// if our window isn't in a
		if (!scene)
			return;

		UIInterfaceOrientationMask interfaceOrientations = [self supportedInterfaceOrientations];
		UIWindowSceneGeometryPreferencesIOS* geometryPrefs = [[UIWindowSceneGeometryPreferencesIOS alloc] initWithInterfaceOrientations:interfaceOrientations];
		[scene requestGeometryUpdateWithPreferences:geometryPrefs errorHandler:^(NSError* error) {
			NSLog(@"***** iOS 16 rotation error=%@", error);
		}];
    } else {
        // Frowned upon, but it works...
        NSNumber* orientationValue = [NSNumber numberWithInt:(int)deviceOrientation];
        [UIDevice.currentDevice setValue:orientationValue forKey:@"orientation"];
        [UIViewController attemptRotationToDeviceOrientation];
    }
}

- (void)updateProperties
{
    if (!isQtApplication())
        return;

    if (!self.platformScreen || !self.platformScreen->screen())
        return;

    // For now we only care about the main screen, as both the statusbar
    // visibility and orientation is only appropriate for the main screen.
    if (self.platformScreen->uiScreen() != [UIScreen mainScreen])
        return;

    // Prevent recursion caused by updating the status bar appearance (position
    // or visibility), which in turn may cause a layout of our subviews, and
    // a reset of window-states, which themselves affect the view controller
    // properties such as the statusbar visibility.
    if (m_updatingProperties)
        return;

    QScopedValueRollback<BOOL> updateRollback(m_updatingProperties, YES);

    QWindow *focusWindow = QGuiApplication::focusWindow();

    // If we don't have a focus window we leave the statusbar
    // as is, so that the user can activate a new window with
    // the same window state without the status bar jumping
    // back and forth.
    if (!focusWindow)
        return;

    // We only care about changes to focusWindow that involves our screen
    if (!focusWindow->screen() || focusWindow->screen()->handle() != self.platformScreen)
        return;

    // All decisions are based on the top level window
    focusWindow = qt_window_private(focusWindow)->topLevelWindow();

#ifndef Q_OS_TVOS

    // -------------- Status bar style and visbility ---------------

    UIStatusBarStyle oldStatusBarStyle = self.preferredStatusBarStyle;
    if (focusWindow->flags() & Qt::MaximizeUsingFullscreenGeometryHint)
        self.preferredStatusBarStyle = UIStatusBarStyleDefault;
    else
        self.preferredStatusBarStyle = UIStatusBarStyleLightContent;

    if (self.preferredStatusBarStyle != oldStatusBarStyle)
        [self setNeedsStatusBarAppearanceUpdate];

    bool currentStatusBarVisibility = self.prefersStatusBarHidden;
    self.prefersStatusBarHidden = focusWindow->windowState() == Qt::WindowFullScreen;

    if (self.prefersStatusBarHidden != currentStatusBarVisibility) {
        [self setNeedsStatusBarAppearanceUpdate];
        [self.view setNeedsLayout];
    }


    // -------------- Content orientation ---------------

    Qt::ScreenOrientation contentOrientation = focusWindow->contentOrientation();
    if (contentOrientation != Qt::PrimaryOrientation) {
        UIInterfaceOrientation uiOrientation = UIInterfaceOrientationPortrait;
        switch (contentOrientation) {
        case Qt::LandscapeOrientation:
            uiOrientation = UIInterfaceOrientationLandscapeRight;
            break;
        case Qt::InvertedLandscapeOrientation:
            uiOrientation = UIInterfaceOrientationLandscapeLeft;
            break;
        case Qt::InvertedPortraitOrientation:
            uiOrientation = UIInterfaceOrientationPortraitUpsideDown;
            break;
        case Qt::PortraitOrientation:
        default:
            uiOrientation = UIInterfaceOrientationPortrait;
            break;
        }
        
        // if orientation wasn't previously locked, then note the original orientation
        if (!self.lockedOrientation)
            self.preLockedOrienation = UIDevice.currentDevice.orientation;

        [self updateToOrientation:uiOrientation deviceOrientation:fromQtScreenOrientation(contentOrientation)];
    } else {
        // The content orientation is set to Qt::PrimaryOrientation, meaning
        // that auto-rotation should be enabled. But we may be coming out of
        // a state of locked orientation, which needs some cleanup before we
        // can enable auto-rotation again.
        if (self.lockedOrientation) {
            // Then we can re-enable auto-rotation and try to go back to the pre-locked orientation if possible
            [self updateToOrientation:UIInterfaceOrientationUnknown deviceOrientation:self.preLockedOrienation];
        }
    }
#endif
}

- (NSArray*)keyCommands
{
    // FIXME: If we are on iOS 13.4 or later we can use UIKey instead of doing this
    // So it should be safe to remove this entire function and handleShortcut() as
    // a result
    NSMutableArray<UIKeyCommand *> *keyCommands = nil;
    QShortcutMap &shortcutMap = QGuiApplicationPrivate::instance()->shortcutMap;
    keyCommands = [[NSMutableArray<UIKeyCommand *> alloc] init];
    const QList<QKeySequence> keys = shortcutMap.keySequences();
    for (const QKeySequence &seq : keys) {
        const QString keyString = seq.toString();
        [keyCommands addObject:[UIKeyCommand
            keyCommandWithInput:QString(keyString[keyString.length() - 1]).toNSString()
            modifierFlags:QAppleKeyMapper::toUIKitModifiers(seq[0].keyboardModifiers())
            action:@selector(handleShortcut:)]];
    }
    return keyCommands;
}

- (void)handleShortcut:(UIKeyCommand *)keyCommand
{
    const QString str = QString::fromNSString([keyCommand input]);
    Qt::KeyboardModifiers qtMods = QAppleKeyMapper::fromUIKitModifiers(keyCommand.modifierFlags);
    QChar ch = str.isEmpty() ? QChar() : str.at(0);
    QShortcutMap &shortcutMap = QGuiApplicationPrivate::instance()->shortcutMap;
    QKeyEvent keyEvent(QEvent::ShortcutOverride, Qt::Key(ch.toUpper().unicode()), qtMods, str);
    shortcutMap.tryShortcut(&keyEvent);
}



@end

