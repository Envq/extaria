QT += core network

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
    supervisor.cpp

HEADERS += \
    supervisor.h

unix: LIBS += -L$$OUT_PWD/../Utils/ -lUtils
INCLUDEPATH += $$PWD/../Utils
DEPENDPATH += $$PWD/../Utils
unix: PRE_TARGETDEPS += $$OUT_PWD/../Utils/libUtils.a

unix: LIBS += -L$$OUT_PWD/../TouchNode/ -lTouchNode
INCLUDEPATH += $$PWD/../TouchNode
DEPENDPATH += $$PWD/../TouchNode
unix: PRE_TARGETDEPS += $$OUT_PWD/../TouchNode/libTouchNode.a

unix: LIBS += -L$$OUT_PWD/../MecaNode/ -lMecaNode
INCLUDEPATH += $$PWD/../MecaNode
DEPENDPATH += $$PWD/../MecaNode
unix: PRE_TARGETDEPS += $$OUT_PWD/../MecaNode/libMecaNode.a

# Default rules for deployment.
unix {
    target.path = $$[QT_INSTALL_PLUGINS]/generic
}
!isEmpty(target.path): INSTALLS += target
