TEMPLATE=subdirs
SUBDIRS=\
    qfactoryloader \
    quuid \
    qpluginloader

qtConfig(library): SUBDIRS += \
    qplugin \
    qlibrary

contains(CONFIG, static) {
    message(Disabling tests requiring shared build of Qt)
    SUBDIRS -= qplugin
}
