#include "logs.h"

#include "settings.h"

#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QTextStream>
#include <QThread>

Q_LOGGING_CATEGORY(logLogger, "Logger")

// ==========================================================================
teleop::Logger::Logger(const QString& name, unsigned buffer_size,
                       QObject* parent)
    : QObject(parent), _name(name), _bufferSize(buffer_size) {
    qDebug(logLogger()) << QThread::currentThreadId() << " | Logger::Logger"
                        << _name;
    // Create Logs Dir if not exist
    QString dir_path =
        SettingsManager::getInstance().getDirectoryPath() + "/logs/";
    QDir dir;
    if (!dir.exists(dir_path)) {
        dir.mkpath(dir_path);
    }
    // Set the logging file (create it if not exist)
    _file = new QFile(dir_path + "log_" + _name + ".txt");
    _file->open(QFile::WriteOnly);
    // Set the buffer
    _buffer = new QString[_bufferSize];
}


teleop::Logger::~Logger() {
    qDebug(logLogger()) << QThread::currentThreadId() << " | Logger::~Logger"
                        << _name;
    flush();
    _file->close();
    delete[] _buffer;
    delete _file;
}


void teleop::Logger::write(const QVector<float>& vec) {
    // unsigned=32bit -> 2**32-1=4294967295 -> 1193h with sampling_rate=1000hz
    auto timestamp = QString::number(LogClock::getInstance().getNanoseconds());
    //    auto timestamp =
    //    QDateTime::currentDateTime().toString("hh:mm:ss.zzz");
    auto msg =
        QString("%0|%1|%2|%3|%4|%5|%6\n")
            .arg(timestamp, QString::number(vec[0]), QString::number(vec[1]),
                 QString::number(vec[2]), QString::number(vec[3]),
                 QString::number(vec[4]), QString::number(vec[5]));

    if (_bufferIndex >= _bufferSize) {
        qDebug(logLogger())
            << "Logger" << _name << "Buffer overflow: " << _bufferIndex << "on"
            << _bufferSize;
        qDebug(logLogger) << "FATAL";
        flush();
        exit(EXIT_FAILURE);
    } else {
        _buffer[_bufferIndex] = msg;
        _bufferIndex++;
    }
}


void teleop::Logger::flush() {
    QTextStream logStream(_file);
    for (qint64 i = 0; i < _bufferSize; ++i) {
        logStream << _buffer[i];
    }
    logStream.flush();
}


// ==========================================================================
teleop::LogClock& teleop::LogClock::getInstance() {
    static LogClock instance;
    return instance;
}


void teleop::LogClock::start() {
    if (!_timer.isValid()) {
        _timer.start();
    }
}


bool teleop::LogClock::isValid() {
    return _timer.isValid();
}


quint64 teleop::LogClock::getNanoseconds() {
    return _timer.nsecsElapsed();
}


float teleop::LogClock::getMilliseconds() {
    return _timer.nsecsElapsed() * 1e-6;
}


float teleop::LogClock::getSeconds() {
    return _timer.nsecsElapsed() * 1e-9;
}


// ==========================================================================
void teleop::messageHandler(QtMsgType type, const QMessageLogContext& context,
                            const QString& msg) {
    char category[] = "               ";
    int  dim        = std::min(strlen(category), strlen(context.category));
    for (int i = 0; i < dim; ++i) {
        category[i] = context.category[i];
    }
    switch (type) {
        case QtInfoMsg:
            fprintf(stderr, "[INFO    ][%s] %s\n", category,
                    msg.toLocal8Bit().constData());
            break;
        case QtDebugMsg:
            fprintf(stderr, "[DEBUG   ][%s] %s\n", category,
                    msg.toLocal8Bit().constData());
            break;
        case QtWarningMsg:
            fprintf(stderr, "[WARNING ][%s] %s\n", category,
                    msg.toLocal8Bit().constData());
            break;
        case QtCriticalMsg:
            fprintf(stderr, "[CRITICAL][%s] %s\n", category,
                    msg.toLocal8Bit().constData());
            break;
        case QtFatalMsg:
            fprintf(stderr, "[FATAL   ][%s] %s\n", category,
                    msg.toLocal8Bit().constData());
            exit(EXIT_FAILURE);
            break;
    }
}
