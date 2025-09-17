// Copyright (C) 2023 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial

package org.qtproject.qt.android;

import android.app.Activity;
import android.content.Context;
import android.content.res.Configuration;
import android.content.res.Resources;
import android.content.res.TypedArray;
import android.graphics.Rect;
import android.hardware.display.DisplayManager;
import android.os.Build;
import android.util.DisplayMetrics;
import android.util.Size;
import android.view.Display;
import android.view.Surface;
import android.view.View;
import android.view.WindowInsets;
import android.view.WindowManager;
import android.view.WindowManager.LayoutParams;
import android.view.WindowMetrics;
import android.view.WindowInsetsController;
import android.view.Window;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import android.graphics.Color;
import android.util.TypedValue;
import android.content.res.Resources.Theme;

import android.util.Log;

class QtDisplayManager
{
    private static String QtTAG = "QtDisplayManager";
    // screen methods
    static native void setDisplayMetrics(int screenWidthPixels, int screenHeightPixels,
                                                int availableLeftPixels, int availableTopPixels,
                                                int availableWidthPixels, int availableHeightPixels,
                                                double XDpi, double YDpi, double scaledDensity,
                                                double density, float refreshRate);
    static native void handleOrientationChanged(int newRotation, int nativeOrientation);
    static native void handleRefreshRateChanged(float refreshRate);
    static native void handleUiDarkModeChanged(int newUiMode);
    static native void handleScreenAdded(int displayId);
    static native void handleScreenChanged(int displayId);
    static native void handleScreenRemoved(int displayId);
    // screen methods

    private boolean m_isFullScreen = false;
    private boolean m_expandedToCutout = false;

    private static int m_previousRotation = -1;

    private final DisplayManager.DisplayListener m_displayListener;
    private final Activity m_activity;

    QtDisplayManager(Activity activity)
    {
        m_activity = activity;
        m_displayListener = new DisplayManager.DisplayListener() {
            @Override
            public void onDisplayAdded(int displayId) {
                QtDisplayManager.handleScreenAdded(displayId);
            }

            @Override
            public void onDisplayChanged(int displayId) {
                Display display = (Build.VERSION.SDK_INT < Build.VERSION_CODES.R)
                        ? m_activity.getWindowManager().getDefaultDisplay()
                        : m_activity.getDisplay();
                float refreshRate = getRefreshRate(display);
                QtDisplayManager.handleRefreshRateChanged(refreshRate);
                QtDisplayManager.handleScreenChanged(displayId);
            }

            @Override
            public void onDisplayRemoved(int displayId) {
                QtDisplayManager.handleScreenRemoved(displayId);
            }
        };
    }

    static void handleOrientationChanges(Activity activity)
    {
        int currentRotation = getDisplayRotation(activity);
        if (m_previousRotation == currentRotation)
            return;
        int nativeOrientation = getNativeOrientation(activity, currentRotation);
        QtDisplayManager.handleOrientationChanged(currentRotation, nativeOrientation);
        m_previousRotation = currentRotation;
    }

    static int getDisplayRotation(Activity activity) {
        Display display = Build.VERSION.SDK_INT < Build.VERSION_CODES.R ?
                activity.getWindowManager().getDefaultDisplay() :
                activity.getDisplay();

        return display != null ? display.getRotation() : 0;
    }

    private static int getNativeOrientation(Activity activity, int rotation)
    {
        int orientation = activity.getResources().getConfiguration().orientation;
        boolean rot90 = (rotation == Surface.ROTATION_90 || rotation == Surface.ROTATION_270);
        boolean isLandscape = (orientation == Configuration.ORIENTATION_LANDSCAPE);
        if ((isLandscape && !rot90) || (!isLandscape && rot90))
            return Configuration.ORIENTATION_LANDSCAPE;

        return Configuration.ORIENTATION_PORTRAIT;
    }

    static float getRefreshRate(Display display)
    {
        return display != null ? display.getRefreshRate() : 60.0f;
    }

    void registerDisplayListener()
    {
        DisplayManager displayManager =
                (DisplayManager) m_activity.getSystemService(Context.DISPLAY_SERVICE);
        displayManager.registerDisplayListener(m_displayListener, null);
    }

    void unregisterDisplayListener()
    {
        DisplayManager displayManager =
                (DisplayManager) m_activity.getSystemService(Context.DISPLAY_SERVICE);
        displayManager.unregisterDisplayListener(m_displayListener);
    }

    void setSystemUiVisibility(boolean isFullScreen, boolean expandedToCutout)
    {
        if (m_isFullScreen == isFullScreen && m_expandedToCutout == expandedToCutout)
            return;

        m_isFullScreen = isFullScreen;
        m_expandedToCutout = expandedToCutout;
        Window window = m_activity.getWindow();
        View decorView = window.getDecorView();

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            int cutoutMode;
            if (m_isFullScreen || m_expandedToCutout) {
                window.setDecorFitsSystemWindows(false);
                cutoutMode = LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_SHORT_EDGES;
            } else {
                window.setDecorFitsSystemWindows(true);
                cutoutMode = LayoutParams.LAYOUT_IN_DISPLAY_CUTOUT_MODE_DEFAULT;
            }
            LayoutParams layoutParams = window.getAttributes();
            layoutParams.layoutInDisplayCutoutMode = cutoutMode;
            window.setAttributes(layoutParams);

            final WindowInsetsController insetsControl = window.getInsetsController();
            if (insetsControl != null) {
                int sysBarsBehavior;
                if (m_isFullScreen) {
                    insetsControl.hide(WindowInsets.Type.systemBars());
                    sysBarsBehavior = WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE;
                } else {
                    insetsControl.show(WindowInsets.Type.systemBars());
                    sysBarsBehavior = WindowInsetsController.BEHAVIOR_DEFAULT;
                }
                insetsControl.setSystemBarsBehavior(sysBarsBehavior);
            }


        } else {
            int systemUiVisibility;

            if (m_isFullScreen || m_expandedToCutout) {
                systemUiVisibility =  View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                                    | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION;
                if (m_isFullScreen) {
                    systemUiVisibility |=  View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                                        | View.SYSTEM_UI_FLAG_FULLSCREEN
                                        | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                                        | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY;
                }
            } else {
                systemUiVisibility = View.SYSTEM_UI_FLAG_VISIBLE;
            }

            decorView.setSystemUiVisibility(systemUiVisibility);
        }

        if (!isFullScreen && !edgeToEdgeEnabled(m_activity)) {
            // These are needed to operate on system bar colors
            window.clearFlags(WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS
                        | WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION);
            window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);

            // Handle transparent status and navigation bars
            if (m_expandedToCutout) {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
                    window.setStatusBarColor(Color.TRANSPARENT);
                    window.setNavigationBarColor(Color.TRANSPARENT);
                } else {
                    // Android 9 and prior doesn't add the semi-transparent bars
                    // to avoid low contrast system icons, so try to mimick it
                    // by taking the current color and only increase the opacity.
                    int statusBarColor = window.getStatusBarColor();
                    int transparentStatusBar = statusBarColor & 0x00FFFFFF;
                    window.setStatusBarColor(transparentStatusBar);

                    int navigationBarColor = window.getNavigationBarColor();
                    int semiTransparentNavigationBar = navigationBarColor & 0x7FFFFFFF;
                    window.setNavigationBarColor(semiTransparentNavigationBar);
                }
            } else {
                // Restore theme's system bars colors
                Theme theme = m_activity.getTheme();
                TypedValue typedValue = new TypedValue();

                theme.resolveAttribute(android.R.attr.statusBarColor, typedValue, true);
                int defaultStatusBarColor = typedValue.data;
                window.setStatusBarColor(defaultStatusBarColor);

                theme.resolveAttribute(android.R.attr.navigationBarColor, typedValue, true);
                int defaultNavigationBarColor = typedValue.data;
                window.setNavigationBarColor(defaultNavigationBarColor);
            }
        }

        decorView.post(() -> decorView.requestApplyInsets());
    }

    private static boolean edgeToEdgeEnabled(Activity activity) {
        if (Build.VERSION.SDK_INT > Build.VERSION_CODES.VANILLA_ICE_CREAM)
            return true;
        if (Build.VERSION.SDK_INT < Build.VERSION_CODES.VANILLA_ICE_CREAM)
            return false;
        int[] attrs = new int[] { android.R.attr.windowOptOutEdgeToEdgeEnforcement };
        TypedArray ta = activity.getTheme().obtainStyledAttributes(attrs);
        try {
            return !ta.getBoolean(0, false);
        } finally {
            ta.recycle();
        }
    }

    boolean isFullScreen()
    {
        return m_isFullScreen;
    }

    boolean expandedToCutout()
    {
        return m_expandedToCutout;
    }

    boolean decorFitsSystemWindows()
    {
        return !isFullScreen() && !expandedToCutout();
    }

    void reinstateFullScreen()
    {
        if (m_isFullScreen) {
            m_isFullScreen = false;
            setSystemUiVisibility(true, m_expandedToCutout);
        }
    }

    /*
    * Convenience method to call deprecated API prior to Android R (30).
    */
    @SuppressWarnings ("deprecation")
    private static void setSystemUiVisibility(View decorView, int flags)
    {
        decorView.setSystemUiVisibility(flags);
    }

    /*
    * Set the status bar color scheme hint so that the system decides how to color the icons.
    */
    @UsedFromNativeCode
    static void setStatusBarColorHint(Activity activity, boolean isLight)
    {
        Window window = activity.getWindow();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            WindowInsetsController controller = window.getInsetsController();
            if (controller != null) {
                int lightStatusBarMask = WindowInsetsController.APPEARANCE_LIGHT_STATUS_BARS;
                int appearance = isLight ? lightStatusBarMask : 0;
                controller.setSystemBarsAppearance(appearance, lightStatusBarMask);
            }
        } else {
            @SuppressWarnings("deprecation")
            int currentFlags = window.getDecorView().getSystemUiVisibility();
            @SuppressWarnings("deprecation")
            int lightStatusBarMask = View.SYSTEM_UI_FLAG_LIGHT_STATUS_BAR;
            int appearance = isLight
                ? currentFlags | lightStatusBarMask
                : currentFlags & ~lightStatusBarMask;
            setSystemUiVisibility(window.getDecorView(), appearance);
        }
    }

    /*
    * Set the navigation bar color scheme hint so that the system decides how to color the icons.
    */
    @UsedFromNativeCode
    static void setNavigationBarColorHint(Activity activity, boolean isLight)
    {
        Window window = activity.getWindow();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.R) {
            WindowInsetsController controller = window.getInsetsController();
            if (controller != null) {
                int lightNavigationBarMask = WindowInsetsController.APPEARANCE_LIGHT_NAVIGATION_BARS;
                int appearance = isLight ? lightNavigationBarMask : 0;
                controller.setSystemBarsAppearance(appearance, lightNavigationBarMask);
            }
        } else {
            @SuppressWarnings("deprecation")
            int currentFlags = window.getDecorView().getSystemUiVisibility();
            @SuppressWarnings("deprecation")
            int lightNavigationBarMask = View.SYSTEM_UI_FLAG_LIGHT_NAVIGATION_BAR;
            int appearance = isLight
                ? currentFlags | lightNavigationBarMask
                : currentFlags & ~lightNavigationBarMask;
            setSystemUiVisibility(window.getDecorView(), appearance);
        }
    }

    static int resolveColorAttribute(Activity activity, int attribute)
    {
        Theme theme = activity.getTheme();
        Resources resources = activity.getResources();
        TypedValue tv = new TypedValue();

        if (theme.resolveAttribute(attribute, tv, true)) {
            if (tv.resourceId != 0)
                return resources.getColor(tv.resourceId, theme);
            if (tv.type >= TypedValue.TYPE_FIRST_COLOR_INT && tv.type <= TypedValue.TYPE_LAST_COLOR_INT)
                return tv.data;
        }

        return -1;
    }

    @SuppressWarnings("deprecation")
    static int getThemeDefaultStatusBarColor(Activity activity)
    {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.VANILLA_ICE_CREAM)
            return -1;
        return resolveColorAttribute(activity, android.R.attr.statusBarColor);
    }

    @SuppressWarnings("deprecation")
    static int getThemeDefaultNavigationBarColor(Activity activity)
    {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.VANILLA_ICE_CREAM)
            return -1;
        return resolveColorAttribute(activity, android.R.attr.navigationBarColor);
    }

    static void enableSystemBarsBackgroundDrawing(Window window)
    {
        // These are needed to operate on system bar colors
        window.addFlags(WindowManager.LayoutParams.FLAG_DRAWS_SYSTEM_BAR_BACKGROUNDS);
        @SuppressWarnings("deprecation")
        final int translucentFlags =  WindowManager.LayoutParams.FLAG_TRANSLUCENT_STATUS
                                    | WindowManager.LayoutParams.FLAG_TRANSLUCENT_NAVIGATION;
        window.clearFlags(translucentFlags);
    }

    @SuppressWarnings("deprecation")
    static void setStatusBarColor(Window window, int color)
    {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.VANILLA_ICE_CREAM)
            return;
        window.setStatusBarColor(color);
    }

    @SuppressWarnings("deprecation")
    static void setNavigationBarColor(Window window, int color)
    {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.VANILLA_ICE_CREAM)
            return;
        window.setNavigationBarColor(color);
    }

    @UsedFromNativeCode
    static Display getDisplay(Context context, int displayId)
    {
        DisplayManager displayManager =
                (DisplayManager)context.getSystemService(Context.DISPLAY_SERVICE);
        if (displayManager != null) {
            return displayManager.getDisplay(displayId);
        }
        return null;
    }

    @UsedFromNativeCode
    static List<Display> getAvailableDisplays(Context context)
    {
        DisplayManager displayManager =
                (DisplayManager)context.getSystemService(Context.DISPLAY_SERVICE);
        if (displayManager != null) {
            Display[] displays = displayManager.getDisplays();
            return Arrays.asList(displays);
        }
        return new ArrayList<>();
    }

    @UsedFromNativeCode
    static Size getDisplaySize(Context displayContext, Display display)
    {
        if (display == null || displayContext == null)
            return new Size(0, 0);

        if (android.os.Build.VERSION.SDK_INT < Build.VERSION_CODES.R) {
            final DisplayMetrics metrics = new DisplayMetrics();
            display.getRealMetrics(metrics);
            return new Size(metrics.widthPixels, metrics.heightPixels);
        } else {
            try {
                WindowManager windowManager = displayContext.getSystemService(WindowManager.class);
                if (windowManager != null) {
                    WindowMetrics metrics = windowManager.getCurrentWindowMetrics();
                    Rect areaBounds = metrics.getBounds();
                    return new Size(areaBounds.width(), areaBounds.height());
                } else {
                    Log.e(QtTAG, "getDisplaySize(): WindowManager null, display ID" + display.getDisplayId());
                }
            } catch (Exception e) {
                Log.e(QtTAG, "Failed to retrieve display metrics with " + e);
            }
            return new Size(0, 0);
        }
    }

    static void setApplicationDisplayMetrics(Activity activity, int width, int height)
    {
        if (activity == null)
            return;

        final WindowInsets rootInsets = activity.getWindow().getDecorView().getRootWindowInsets();
        final WindowManager windowManager = activity.getWindowManager();
        Display display;

        int insetLeft;
        int insetTop;

        int maxWidth;
        int maxHeight;

        if (android.os.Build.VERSION.SDK_INT < Build.VERSION_CODES.R) {
            display = windowManager.getDefaultDisplay();

            final DisplayMetrics maxMetrics = new DisplayMetrics();
            display.getRealMetrics(maxMetrics);
            maxWidth = maxMetrics.widthPixels;
            maxHeight = maxMetrics.heightPixels;

            insetLeft = rootInsets.getStableInsetLeft();
            insetTop = rootInsets.getStableInsetTop();
        } else {
            display = activity.getDisplay();

            final WindowMetrics maxMetrics = windowManager.getMaximumWindowMetrics();
            maxWidth = maxMetrics.getBounds().width();
            maxHeight = maxMetrics.getBounds().height();

            insetLeft = rootInsets.getInsetsIgnoringVisibility(WindowInsets.Type.systemBars()).left;
            insetTop = rootInsets.getInsetsIgnoringVisibility(WindowInsets.Type.systemBars()).top;
        }

        final DisplayMetrics displayMetrics = activity.getResources().getDisplayMetrics();

        double density = displayMetrics.density;
        double scaledDensity = displayMetrics.scaledDensity;

        setDisplayMetrics(maxWidth, maxHeight, insetLeft, insetTop,
                width, height, getXDpi(displayMetrics), getYDpi(displayMetrics),
                scaledDensity, density, getRefreshRate(display));
    }

    static float getXDpi(final DisplayMetrics metrics) {
        if (metrics.xdpi < android.util.DisplayMetrics.DENSITY_LOW)
            return android.util.DisplayMetrics.DENSITY_LOW;
        return metrics.xdpi;
    }

    static float getYDpi(final DisplayMetrics metrics) {
        if (metrics.ydpi < android.util.DisplayMetrics.DENSITY_LOW)
            return android.util.DisplayMetrics.DENSITY_LOW;
        return metrics.ydpi;
    }
}
