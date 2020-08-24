TEMPLATE = subdirs
QT_FOR_CONFIG += gui-private

qtHaveModule(dbus) {
    !vxworks:!mac:!win32:SUBDIRS += ibus
}

qtConfig(xcb): SUBDIRS += compose


