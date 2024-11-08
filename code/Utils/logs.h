#ifndef LOGS_H
#define LOGS_H

#include <QElapsedTimer>
#include <QFile>
#include <QLoggingCategory>
#include <QObject>
#include <QVector3D>

Q_DECLARE_LOGGING_CATEGORY(logLogger)


namespace teleop {

// ==========================================================================
class LogClock {  // singleton
  public:
    static LogClock& getInstance();

    void    start();
    bool    isValid();
    quint64 getNanoseconds();
    float   getMilliseconds();
    float   getSeconds();

  private:
    QElapsedTimer _timer;

    LogClock()                = default;
    LogClock(const LogClock&) = delete;
    void operator=(const LogClock&) = delete;
};

// ==========================================================================
class Logger : public QObject {
    Q_OBJECT

  public:
    Logger(const QString& name, unsigned buffer_size,
           QObject* parent = nullptr);
    Logger(const Logger&) = delete;
    Logger(Logger&&)      = delete;
    ~Logger();

  private:
    const QString  _name        = "Logger";
    QFile*         _file        = nullptr;
    QString*       _buffer      = nullptr;
    const unsigned _bufferSize  = 1000;
    unsigned       _bufferIndex = 0;

  public slots:
    void write(const QVector<float>& vec);
    void flush();
};

// ==========================================================================
void messageHandler(QtMsgType type, const QMessageLogContext& context,
                    const QString& msg);

}  // namespace teleop


#endif  // LOGS_H
