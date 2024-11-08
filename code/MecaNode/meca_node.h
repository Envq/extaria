#ifndef MECA_NODE_H
#define MECA_NODE_H

#include "meca_adapter.h"
#include "settings.h"

#include <QLoggingCategory>
#include <QMutex>
#include <QObject>
#include <QThread>
#include <QTimer>

Q_DECLARE_LOGGING_CATEGORY(logMecaNode)


namespace teleop {

// ==========================================================================
enum class MecaState { Init, WaitForInit, Start, WaitForStart, Teleop };

// ==========================================================================
struct MecaRequestData {
    bool           fired   = true;
    QVector<float> poseAbs = {0, 0, 0, 0, 0, 0};
    QVector<float> poseRel = {0, 0, 0, 0, 0, 0};
    QVector<float> twist   = {0, 0, 0, 0, 0, 0};
    teleop::Mode   mode    = teleop::Mode::rel;
};

// ==========================================================================
class MecaWorker : public QObject {
    Q_OBJECT

  public:
    MecaWorker();
    MecaWorker(const MecaWorker&) = delete;
    MecaWorker(MecaWorker&&)      = delete;
    ~MecaWorker();

    void setRequest(const MecaRequestData& request);

  private:
    QTimer*                 _loopTimer = nullptr;
    mecademic::MecaAdapter* _meca      = nullptr;
    MecaState               _state     = MecaState::Init;
    QMutex                  _mutex;
    MecaRequestData         _request;

    unsigned       _loopPeriod            = 500;
    QString        _robotIP               = "192.168.0.100";
    float          _blending              = 100;
    float          _jointVelocity         = 25;
    float          _jointAcceleration     = 100;
    float          _velocityTimeout       = 0.05;
    float          _cartesianAcceleration = 50;
    QVector<float> _tcp;
    QVector<float> _origin;
    bool           _logEnabled = false;

  signals:
    void finished();
    void feedback(const QVector<float>& pose, const QVector<float>& twist);
    // logging
    void logRequestPoseAbs(const QVector<float>& poseAbs);
    void logRequestPoseRel(const QVector<float>& poseRel);
    void logRequestTwist(const QVector<float>& twist);
    void logCurrentPose(const QVector<float>& pose);
    void logCurrentTwist(const QVector<float>& twist);

  public slots:
    void onStart();
    void dutyCycle();
    void onFeedback(const QVector<float>& pose, const QVector<float>& twist);
};

// ==========================================================================
class MecaNode : public QObject {
    Q_OBJECT

  public:
    explicit MecaNode(QObject* parent = nullptr);
    MecaNode(const MecaNode&) = delete;
    MecaNode(MecaNode&&)      = delete;
    ~MecaNode();

  private:
    QThread*    _thread = nullptr;
    MecaWorker* _worker = nullptr;

  signals:
    void finished();
    void feedback(const QVector<float>& pose, const QVector<float>& twist);

  public slots:
    void onStart();
    void onRequest(const QVector<float>& poseAbs, const QVector<float>& poseRel,
                   const QVector<float>& twist, const teleop::Mode& mode);
};

}  // namespace teleop

#endif  // MECA_NODE_H
