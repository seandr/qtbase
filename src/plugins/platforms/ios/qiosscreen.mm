// Copyright (C) 2016 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR LGPL-3.0-only OR GPL-2.0-only OR GPL-3.0-only

#undef QT_NO_FOREACH // this file contains unported legacy Q_FOREACH uses

#include "qiosglobal.h"
#include "qiosintegration.h"
#include "qiosscreen.h"
#include "qioswindow.h"
#include <qpa/qwindowsysteminterface.h>
#include "qiosapplicationdelegate.h"
#include "qiosviewcontroller.h"
#include "quiview.h"
#include "qiostheme.h"

#include <QtCore/private/qcore_mac_p.h>

#include <QtGui/qpointingdevice.h>
#include <QtGui/private/qwindow_p.h>
#include <QtGui/private/qguiapplication_p.h>
#include <private/qcoregraphics_p.h>
#include <qpa/qwindowsysteminterface.h>

#include <sys/sysctl.h>

// -------------------------------------------------------------------------

typedef void (^DisplayLinkBlock)(CADisplayLink *displayLink);

@implementation UIScreen (DisplayLinkBlock)
- (CADisplayLink*)displayLinkWithBlock:(DisplayLinkBlock)block
{
    return [self displayLinkWithTarget:[[block copy] autorelease]
        selector:@selector(invokeDisplayLinkBlock:)];
}
@end

@implementation NSObject (DisplayLinkBlock)
- (void)invokeDisplayLinkBlock:(CADisplayLink *)sender
{
    DisplayLinkBlock block = static_cast<id>(self);
    block(sender);
}
@end


// -------------------------------------------------------------------------

static QIOSScreen* qtPlatformScreenFor(UIScreen *uiScreen)
{
    foreach (QScreen *screen, QGuiApplication::screens()) {
        QIOSScreen *platformScreen = static_cast<QIOSScreen *>(screen->handle());
        if (platformScreen->uiScreen() == uiScreen)
            return platformScreen;
    }

    return 0;
}

@interface QIOSScreenTracker : NSObject
@end

@implementation QIOSScreenTracker

+ (void)load
{
    NSNotificationCenter *center = [NSNotificationCenter defaultCenter];
    [center addObserver:self selector:@selector(screenConnected:)
            name:UIScreenDidConnectNotification object:nil];
    [center addObserver:self selector:@selector(screenDisconnected:)
            name:UIScreenDidDisconnectNotification object:nil];
    [center addObserver:self selector:@selector(screenModeChanged:)
            name:UIScreenModeDidChangeNotification object:nil];
}

+ (void)screenConnected:(NSNotification*)notification
{
    if (!QIOSIntegration::instance())
        return; // Will be added when QIOSIntegration is created

    QWindowSystemInterface::handleScreenAdded(new QIOSScreen([notification object]));
}

+ (void)screenDisconnected:(NSNotification*)notification
{
    if (!QIOSIntegration::instance())
        return;

    QIOSScreen *screen = qtPlatformScreenFor([notification object]);
    Q_ASSERT_X(screen, Q_FUNC_INFO, "Screen disconnected that we didn't know about");

    QWindowSystemInterface::handleScreenRemoved(screen);
}

+ (void)screenModeChanged:(NSNotification*)notification
{
    if (!QIOSIntegration::instance())
        return;

    QIOSScreen *screen = qtPlatformScreenFor([notification object]);
    Q_ASSERT_X(screen, Q_FUNC_INFO, "Screen changed that we didn't know about");

    screen->updateProperties();
}

@end

// -------------------------------------------------------------------------

@interface QIOSOrientationListener : NSObject
@end

@implementation QIOSOrientationListener {
    QIOSScreen *m_screen;
}

- (instancetype)initWithQIOSScreen:(QIOSScreen *)screen
{
    self = [super init];
    if (self) {
        m_screen = screen;
#ifndef Q_OS_TVOS
        [[UIDevice currentDevice] beginGeneratingDeviceOrientationNotifications];
        [[NSNotificationCenter defaultCenter]
            addObserver:self
            selector:@selector(orientationChanged:)
            name:@"UIDeviceOrientationDidChangeNotification" object:nil];
#endif
    }
    return self;
}

- (void)dealloc
{
#ifndef Q_OS_TVOS
    [[UIDevice currentDevice] endGeneratingDeviceOrientationNotifications];
    [[NSNotificationCenter defaultCenter]
        removeObserver:self
        name:@"UIDeviceOrientationDidChangeNotification" object:nil];
#endif
    [super dealloc];
}

- (void)orientationChanged:(NSNotification *)notification
{
    Q_UNUSED(notification);
    m_screen->updateRotation();
}

@end

@interface UIScreen (Compatibility)
@property (nonatomic, readonly) CGRect qt_applicationFrame;
@end

@implementation UIScreen (Compatibility)
- (CGRect)qt_applicationFrame
{
    return self.bounds;
}
@end

// -------------------------------------------------------------------------

@implementation QUIWindow {
    QIOSScreen *m_screen;
    BOOL m_observingEffectiveGeometry;
}

- (instancetype)initWithFrame:(CGRect)frame QIOSScreen:(QIOSScreen*)screen
{
    if ((self = [super initWithFrame:frame])) {
        self->_sendingEvent = NO;

        m_screen = screen;
        m_observingEffectiveGeometry = NO;
        if (@available(ios 16, macCatalyst 16.0, tvOS 16, *)) {
			UIWindowSceneGeometry* effectiveGeometry = self.windowScene.effectiveGeometry;
            if (effectiveGeometry) {
                [self.windowScene addObserver:self forKeyPath:@"effectiveGeometry" options:(NSKeyValueObservingOptionNew) context:nullptr];
                m_observingEffectiveGeometry = YES;
            }
            NSLog(@"%@:initwithFrame: m_uiWindow.windowScene=%@ effectiveGeometry=%@", self, self.windowScene, effectiveGeometry);
        }
    }
    
    return self;
}

- (void)dealloc
{
    if (@available(ios 16, macCatalyst 16.0, tvOS 16, *)) {
        if (m_observingEffectiveGeometry && self.windowScene)
            [self.windowScene removeObserver:self forKeyPath:@"effectiveGeometry"];
    }
    [super dealloc];
}

- (void)sendEvent:(UIEvent *)event
{
    QScopedValueRollback<BOOL> sendingEvent(self->_sendingEvent, YES);
    [super sendEvent:event];
}

- (void)traitCollectionDidChange:(UITraitCollection *)previousTraitCollection
{
    [super traitCollectionDidChange:previousTraitCollection];

    if (!qGuiApp)
        return;

    Qt::ColorScheme colorScheme = self.traitCollection.userInterfaceStyle
                              == UIUserInterfaceStyleDark
                              ? Qt::ColorScheme::Dark
                              : Qt::ColorScheme::Light;

    if (self.screen == UIScreen.mainScreen) {
        // Check if the current userInterfaceStyle reports a different appearance than
        // the platformTheme's appearance. We might have set that one based on the UIScreen
        if (previousTraitCollection.userInterfaceStyle != self.traitCollection.userInterfaceStyle
            || QGuiApplicationPrivate::platformTheme()->colorScheme() != colorScheme) {
            QIOSTheme::initializeSystemPalette();
            QWindowSystemInterface::handleThemeChange<QWindowSystemInterface::SynchronousDelivery>();
        }
		if ((self.traitCollection.verticalSizeClass != previousTraitCollection.verticalSizeClass) ||
			(self.traitCollection.horizontalSizeClass != previousTraitCollection.horizontalSizeClass))
		{
//                NSLog(@"%@:traitCollectionDidChange:%@ self.traitCollection=%@", self, previousTraitCollection, self.traitCollection);
			if (m_screen)
				m_screen->updateRotation();
		}
	}
}

- (void)observeValueForKeyPath:(NSString *)keyPath ofObject:(id)object change:(NSDictionary *)change context:(void *)context
{
	NSLog(@"observeValueForKeyPath:%@ ofObject:%@ change=%@", keyPath, object, change);
	if (@available(ios 16, macCatalyst 16.0, tvOS 16, *)) {
		if ((object == self.windowScene) && ([keyPath isEqual:@"effectiveGeometry"]))
		{
			if (m_screen) {
				//                NSLog(@"**** EffectiveGeometry Changed...");
				m_screen->updateRotation();
				UIWindowSceneGeometry* effectiveGeometry = self.windowScene.effectiveGeometry;
				if (effectiveGeometry) {
					NSLog(@"windowScene.effectiveGeometry.interfaceOrientation=%ld", effectiveGeometry.interfaceOrientation);
				}
			}
		}
	}
}

@end

// -------------------------------------------------------------------------

QT_BEGIN_NAMESPACE

using namespace Qt::StringLiterals;

/*!
    Returns the model identifier of the device.
*/
static QString deviceModelIdentifier()
{
#if TARGET_OS_SIMULATOR
    return QString::fromLocal8Bit(qgetenv("SIMULATOR_MODEL_IDENTIFIER"));
#else
    static const char key[] = "hw.machine";

    size_t size;
    sysctlbyname(key, NULL, &size, NULL, 0);

    char value[size];
    sysctlbyname(key, &value, &size, NULL, 0);

    return QString::fromLatin1(QByteArrayView(value, qsizetype(size)));
#endif
}

QIOSScreen::QIOSScreen(UIScreen *screen)
    : QPlatformScreen()
    , m_uiScreen(screen)
    , m_uiWindow(0)
    , m_orientationListener(0)
{
    QString deviceIdentifier = deviceModelIdentifier();

    if (screen == [UIScreen mainScreen] && !deviceIdentifier.startsWith("AppleTV")) {
        // Based on https://en.wikipedia.org/wiki/List_of_iOS_devices#Display

        // iPhone (1st gen), 3G, 3GS, and iPod Touch (1st–3rd gen) are 18-bit devices
        static QRegularExpression lowBitDepthDevices("^(iPhone1,[12]|iPhone2,1|iPod[1-3],1)$");
        m_depth = deviceIdentifier.contains(lowBitDepthDevices) ? 18 : 24;

        static QRegularExpression iPhoneXModels("^iPhone(10,[36])$");
        static QRegularExpression iPhonePlusModels("^iPhone(7,1|8,2|9,[24]|10,[25])$");
        static QRegularExpression iPadMiniModels("^iPad(2,[567]|4,[4-9]|5,[12])$");

        if (deviceIdentifier.contains(iPhoneXModels)) {
            m_physicalDpi = 458;
        } else if (deviceIdentifier.contains(iPhonePlusModels)) {
            m_physicalDpi = 401;
        } else if (deviceIdentifier.startsWith("iPad")) {
            if (deviceIdentifier.contains(iPadMiniModels))
                m_physicalDpi = 163 * devicePixelRatio();
            else
                m_physicalDpi = 132 * devicePixelRatio();
        } else {
            // All normal iPhones, and iPods
            m_physicalDpi = 163 * devicePixelRatio();
        }
    } else {
        // External display, hard to say
        m_depth = 24;
        m_physicalDpi = 96;
    }

    if (!qt_apple_isApplicationExtension()) {
        for (UIWindow *existingWindow in qt_apple_sharedApplication().windows) {
            if (existingWindow.screen == m_uiScreen) {
                m_uiWindow = [existingWindow retain];
                break;
            }
        }

        if (!m_uiWindow) {
            // Create a window and associated view-controller that we can use
            m_uiWindow = [[QUIWindow alloc] initWithFrame:[m_uiScreen bounds] QIOSScreen:this];
            m_uiWindow.rootViewController = [[[QIOSViewController alloc] initWithQIOSScreen:this] autorelease];
        }
    }

    m_orientationListener = [[QIOSOrientationListener alloc] initWithQIOSScreen:this];

    updateProperties();

    m_displayLink = [m_uiScreen displayLinkWithBlock:^(CADisplayLink *) { deliverUpdateRequests(); }];
    m_displayLink.paused = YES; // Enabled when clients call QWindow::requestUpdate()
    [m_displayLink addToRunLoop:[NSRunLoop mainRunLoop] forMode:NSDefaultRunLoopMode];
}

QIOSScreen::~QIOSScreen()
{
    [m_displayLink invalidate];

    [m_orientationListener release];
    [m_uiWindow release];
}

QString QIOSScreen::name() const
{
    if (m_uiScreen == [UIScreen mainScreen])
        return QString::fromNSString([UIDevice currentDevice].model) + " built-in display"_L1;
    else
        return "External display"_L1;
}

void QIOSScreen::updateRotation()
{
//    NSLog(@"updateRotation()");
    // At construction time, we don't yet have an associated QScreen, but we still want
    // to compute the properties above so they are ready for when the QScreen attaches.
    // Also, at destruction time the QScreen has already been torn down, so notifying
    // Qt about changes to the screen will cause asserts in the event delivery system.
    if (!screen())
        return;

    if (screen()->orientation() != orientation())
        QWindowSystemInterface::handleScreenOrientationChange(screen(), orientation());
}

void QIOSScreen::updateGeometry()
{
//    NSLog(@"updateGeometry()");
    QRect previousGeometry = m_geometry;
    QRect previousAvailableGeometry = m_availableGeometry;

    m_geometry = QRectF::fromCGRect(m_uiScreen.bounds).toRect();
	
	NSLog(@"m_uiWindow=%@ m_uiWindow.windowScene=%@ interfaceOrientation=%ld", m_uiWindow, m_uiWindow.windowScene, m_uiWindow.windowScene.interfaceOrientation);
	if (@available(ios 16, macCatalyst 16.0, tvOS 16, *)) {
		NSLog(@"fullScreen=%@ effectiveGeometry=%@", [m_uiWindow.windowScene valueForKey:@"fullScreen"], [m_uiWindow.windowScene valueForKey:@"effectiveGeometry"]);
		UIWindowSceneGeometry* effectiveGeometry = m_uiWindow.windowScene.effectiveGeometry;
		if (effectiveGeometry) {
			NSLog(@"effectiveGeometry.interfaceOrientation=%ld", effectiveGeometry.interfaceOrientation);
		}
	}

    // The application frame doesn't take safe area insets into account, and
    // the safe area insets are not available before the UIWindow is shown,
    // and do not take split-view constraints into account, so we have to
    // combine the two to get the correct available geometry.
    QRect applicationFrame = QRectF::fromCGRect(m_uiScreen.qt_applicationFrame).toRect();
    UIEdgeInsets safeAreaInsets = m_uiWindow.safeAreaInsets;
    m_availableGeometry = m_geometry.adjusted(safeAreaInsets.left, safeAreaInsets.top,
        -safeAreaInsets.right, -safeAreaInsets.bottom).intersected(applicationFrame);

    if (m_geometry != previousGeometry) {
        // We can't use the primaryOrientation of screen(), as we haven't reported the new geometry yet
        Qt::ScreenOrientation primaryOrientation = m_geometry.width() >= m_geometry.height() ?
            Qt::LandscapeOrientation : Qt::PortraitOrientation;

        // On iPhone 6+ devices, or when display zoom is enabled, the render buffer is scaled
        // before being output on the physical display. We have to take this into account when
        // computing the physical size. Note that unlike the native bounds, the physical size
        // follows the primary orientation of the screen.
        const QRectF physicalGeometry = mapBetween(nativeOrientation(), primaryOrientation, QRectF::fromCGRect(m_uiScreen.nativeBounds).toRect());

        static const qreal millimetersPerInch = 25.4;
        m_physicalSize = physicalGeometry.size() / m_physicalDpi * millimetersPerInch;
    }

    // At construction time, we don't yet have an associated QScreen, but we still want
    // to compute the properties above so they are ready for when the QScreen attaches.
    // Also, at destruction time the QScreen has already been torn down, so notifying
    // Qt about changes to the screen will cause asserts in the event delivery system.
    if (!screen())
        return;

    if (m_geometry != previousGeometry || m_availableGeometry != previousAvailableGeometry)
        QWindowSystemInterface::handleScreenGeometryChange(screen(), m_geometry, m_availableGeometry);
}

void QIOSScreen::updateProperties()
{
//    NSLog(@"updateProperties()");
    updateRotation();

	// Note: The screen orientation change and the geometry changes are not atomic, so when
	// the former is emitted, the latter has not been reported and reflected in the QScreen
	// API yet. But conceptually it makes sense that the orientation update happens first,
	// and the geometry updates caused by auto-rotation happen after that.
    updateGeometry();
}

void QIOSScreen::setUpdatesPaused(bool paused)
{
    m_displayLink.paused = paused;
}

void QIOSScreen::deliverUpdateRequests() const
{
    bool pauseUpdates = true;

    QList<QWindow*> windows = QGuiApplication::allWindows();
    for (int i = 0; i < windows.size(); ++i) {
        QWindow *window = windows.at(i);
        if (platformScreenForWindow(window) != this)
            continue;

        QPlatformWindow *platformWindow = window->handle();
        if (!platformWindow)
            continue;

        if (!platformWindow->hasPendingUpdateRequest())
            continue;

        platformWindow->deliverUpdateRequest();

        // Another update request was triggered, keep the display link running
        if (platformWindow->hasPendingUpdateRequest())
            pauseUpdates = false;
    }

    // Pause the display link if there are no pending update requests
    m_displayLink.paused = pauseUpdates;
}

QRect QIOSScreen::geometry() const
{
    return m_geometry;
}

QRect QIOSScreen::availableGeometry() const
{
    return m_availableGeometry;
}

int QIOSScreen::depth() const
{
    return m_depth;
}

QImage::Format QIOSScreen::format() const
{
    return QImage::Format_ARGB32_Premultiplied;
}

QSizeF QIOSScreen::physicalSize() const
{
    return m_physicalSize;
}

QDpi QIOSScreen::logicalBaseDpi() const
{
    return QDpi(72, 72);
}

qreal QIOSScreen::devicePixelRatio() const
{
    return [m_uiScreen scale];
}

qreal QIOSScreen::refreshRate() const
{
    return m_uiScreen.maximumFramesPerSecond;
}

Qt::ScreenOrientation QIOSScreen::nativeOrientation() const
{
	CGRect nativeBounds =
#if defined(Q_OS_IOS)
		m_uiScreen.nativeBounds;
#else
		m_uiScreen.bounds;
#endif

    // All known iOS devices have a native orientation of portrait, but to
    // be on the safe side we compare the width and height of the bounds.
    return nativeBounds.size.width >= nativeBounds.size.height ?
        Qt::LandscapeOrientation : Qt::PortraitOrientation;
}

Qt::ScreenOrientation QIOSScreen::orientation() const
{
#ifdef Q_OS_TVOS
    return Qt::PrimaryOrientation;
#else
    // Auxiliary screens are always the same orientation as their primary orientation
    if (m_uiScreen != [UIScreen mainScreen])
        return Qt::PrimaryOrientation;

    UIDeviceOrientation deviceOrientation = [UIDevice currentDevice].orientation;

    // At startup, iOS will report an unknown orientation for the device, even
    // if we've asked it to begin generating device orientation notifications.
    // In this case we fall back to the status bar orientation, which reflects
    // the orientation the application was started up in (which may not match
    // the physical orientation of the device, but typically does unless the
    // application has been locked to a subset of the available orientations).
    if (deviceOrientation == UIDeviceOrientationUnknown && !qt_apple_isApplicationExtension())
        deviceOrientation = UIDeviceOrientation(qt_apple_sharedApplication().statusBarOrientation);

    // If the device reports face up or face down orientations, we can't map
    // them to Qt orientations, so we pretend we're in the same orientation
    // as before.
    if (deviceOrientation == UIDeviceOrientationFaceUp || deviceOrientation == UIDeviceOrientationFaceDown) {
        Q_ASSERT(screen());
        return screen()->orientation();
    }

    return toQtScreenOrientation(deviceOrientation);
#endif
}

QPixmap QIOSScreen::grabWindow(WId window, int x, int y, int width, int height) const
{
    if (window && ![reinterpret_cast<id>(window) isKindOfClass:[UIView class]])
        return QPixmap();

    UIView *view = window ? reinterpret_cast<UIView *>(window) : m_uiWindow;

    if (width < 0)
        width = qMax(view.bounds.size.width - x, CGFloat(0));
    if (height < 0)
        height = qMax(view.bounds.size.height - y, CGFloat(0));

    CGRect captureRect = [m_uiWindow convertRect:CGRectMake(x, y, width, height) fromView:view];
    captureRect = CGRectIntersection(captureRect, m_uiWindow.bounds);

    UIGraphicsBeginImageContextWithOptions(captureRect.size, NO, 0.0);
    CGContextRef context = UIGraphicsGetCurrentContext();
    CGContextTranslateCTM(context, -captureRect.origin.x, -captureRect.origin.y);

    // Draws the complete view hierarchy of m_uiWindow into the given rect, which
    // needs to be the same aspect ratio as the m_uiWindow's size. Since we've
    // translated the graphics context, and are potentially drawing into a smaller
    // context than the full window, the resulting image will be a subsection of the
    // full screen.
    [m_uiWindow drawViewHierarchyInRect:m_uiWindow.bounds afterScreenUpdates:NO];

    UIImage *screenshot = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();

    return QPixmap::fromImage(qt_mac_toQImage(screenshot.CGImage));
}

UIScreen *QIOSScreen::uiScreen() const
{
    return m_uiScreen;
}

UIWindow *QIOSScreen::uiWindow() const
{
    return m_uiWindow;
}

QT_END_NAMESPACE

#include "moc_qiosscreen.cpp"
