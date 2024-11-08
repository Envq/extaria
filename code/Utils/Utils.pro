QT += core

TEMPLATE = lib

CONFIG += staticlib
CONFIG += c++11
CONFIG += create_prl
CONFIG(release, debug|release) {
    CONFIG += optimize_full
}

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    logs.cpp \
    kinematic.cpp \
    filters.cpp \
    generators.cpp \
    settings.cpp

HEADERS += \
    logs.h \
    kinematic.h \
    filters.h \
    generators.h \
    settings.h

# Default rules for deployment.
unix {
    target.path = $$[QT_INSTALL_PLUGINS]/generic
}
!isEmpty(target.path): INSTALLS += target

