TEMPLATE = subdirs

SUBDIRS += \
    Utils \
    Supervisor \
    TouchNode \
    MecaNode \
    Main \

Main.depends       = Utils Supervisor
Supervisor.depends = Utils TouchNode MecaNode
TouchNode.depends  = Utils
MecaNode.depends   = Utils

OTHER_FILES += \
    .gitignore \
    .clang-format \
    LICENSE \
    README.md \
    class_diagram.qmodel \
    sequence_diagram.qmodel \
