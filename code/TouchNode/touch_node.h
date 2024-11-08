#ifndef TOUCH_NODE_H
#define TOUCH_NODE_H

#include "touch_adapter.h"

#include <QLoggingCategory>
#include <QMutex>
#include <QObject>
#include <QThread>
#include <QTimer>

Q_DECLARE_LOGGING_CATEGORY(logTouchNode)

namespace teleop {

// ==========================================================================
struct TouchFeedbackData {
    bool           fired  = true;
    QVector<float> wrench = {0, 0, 0, 0, 0, 0};
};

// ==========================================================================
class TouchWorker : public QObject {
    Q_OBJECT

  public:
    TouchWorker();
    TouchWorker(const TouchWorker&) = delete;
    TouchWorker(TouchWorker&&)      = delete;
    ~TouchWorker();

    void setFeedback(const TouchFeedbackData& feedback);

  private:
    QTimer*                  _loopTimer = nullptr;
    systems3d::TouchAdapter* _touch     = nullptr;
    QMutex                   _mutex;
    TouchFeedbackData        _feedback;

    unsigned _loopPeriod      = 500;
    bool     _feedbackEnabled = false;
    bool     _logEnabled      = false;

  signals:
    void finished();
    void request(bool buttonDown, bool buttonUp,
                 const QMatrix4x4& homogeneous_matrix);
    // logging
    void logWrench(const QVector<float>& wrench);

  public slots:
    void onStart();
    void dutyCycle();
};

// ==========================================================================
class TouchNode : public QObject {
    Q_OBJECT

  public:
    explicit TouchNode(QObject* parent = nullptr);
    TouchNode(const TouchNode&) = delete;
    TouchNode(TouchNode&&)      = delete;
    ~TouchNode();

  private:
    QThread*     _thread = nullptr;
    TouchWorker* _worker = nullptr;

  signals:
    void finished();
    void request(bool buttonDown, bool buttonUp,
                 const QMatrix4x4& homogeneous_matrix);

  public slots:
    void onStart();
    void onFeedback(const QVector<float>& wrench);
};

}  // namespace teleop

#endif  // TOUCH_NODE_H
