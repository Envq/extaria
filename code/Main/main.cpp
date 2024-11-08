#include "kinematic.h"
#include "logs.h"
#include "supervisor.h"

#include <QCoreApplication>
#include <QDebug>
#include <QThread>


// ==========================================================================
int main(int argc, char* argv[]) {
    QCoreApplication app(argc, argv);
    setlocale(LC_NUMERIC, "C");  // Fundamental for TouchNode
    QThread::currentThread()->setPriority(QThread::TimeCriticalPriority);
    qDebug() << QThread::currentThreadId() << " | Main";

    qInstallMessageHandler(teleop::messageHandler);
    QLoggingCategory::setFilterRules("*                   = true\n"
                                     "Logger.info         = true\n"
                                     "Kinematic.info      = true\n"
                                     "Supervisor.info     = true\n"
                                     "TouchNode.info      = true\n"
                                     "TouchAdapter.info   = true\n"
                                     "MecaNode.info       = true\n"
                                     "MecaAdapter.info    = true\n"
                                     "Generators.info     = true\n"

                                     "Logger.debug        = false\n"
                                     "Kinematic.debug     = false\n"
                                     "Supervisor.debug    = false\n"
                                     "TouchNode.debug     = false\n"
                                     "TouchAdapter.debug  = false\n"
                                     "MecaNode.debug      = false\n"
                                     "MecaAdapter.debug   = false\n"
                                     "Generators.debug    = false\n");

    teleop::Supervisor brain(&app);
    QObject::connect(&brain, &teleop::Supervisor::finished, &app,
                     &QCoreApplication::quit, Qt::QueuedConnection);
    QObject::connect(&brain, &teleop::Supervisor::finished,
                     []() { qDebug() << "Quit"; });
    brain.start();

    return app.exec();
}
