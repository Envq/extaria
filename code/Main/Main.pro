QT += core network

CONFIG += c++11 console
CONFIG -= app_bundle
CONFIG += link_prl
CONFIG(release, debug|release) {
    CONFIG += optimize_full
}

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
        main.cpp

unix: LIBS += -L$$OUT_PWD/../Utils/ -lUtils
INCLUDEPATH += $$PWD/../Utils
DEPENDPATH += $$PWD/../Utils
unix: PRE_TARGETDEPS += $$OUT_PWD/../Utils/libUtils.a

unix: LIBS += -L$$OUT_PWD/../Supervisor/ -lSupervisor
INCLUDEPATH += $$PWD/../TouchNode
INCLUDEPATH += $$PWD/../MecaNode
INCLUDEPATH += $$PWD/../Supervisor
DEPENDPATH += $$PWD/../Supervisor
unix: PRE_TARGETDEPS += $$OUT_PWD/../Supervisor/libSupervisor.a

# Default rules for deployment.
unix {
    target.path = $$[QT_INSTALL_PLUGINS]/generic
}
!isEmpty(target.path): INSTALLS += target

