#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include "filters.h"
#include "generators.h"
#include "settings.h"

#include "meca_node.h"
#include "touch_node.h"

#include <QLoggingCategory>
#include <QMatrix4x4>
#include <QObject>
#include <QVector>

Q_DECLARE_LOGGING_CATEGORY(logSupervisor)
Q_DECLARE_METATYPE(teleop::Mode)


namespace teleop {

// ==========================================================================
class Supervisor : public QObject {
    Q_OBJECT

  public:
    explicit Supervisor(QObject* parent = nullptr);
    ~Supervisor();

    void start();

  private:
    SettingsPtr            _configs;
    MotionGenerator*       _motionGenerator;
    ForceGenerator*        _forceGenerator;
    SimpleMovingAverage*   _sma;
    WeightedMovingAverage* _wma;
    SimpleMovingMedian*    _smm;
    ButterworthLowPass*    _blp;

    bool         _performFeedback = false;
    bool         _lastPerform     = false;
    unsigned     _logSize;
    bool         _feedbackFromRobot;
    bool         _enableLoggingFilters;
    Mode         _taskMode;
    FeedbackType _feedbackType;
    FilterType   _filterType;

  signals:
    void started();
    void finished();
    void requestForRobot(const QVector<float>& poseAbs,
                         const QVector<float>& poseRel,
                         const QVector<float>& twist, const teleop::Mode& mode);
    void feedbackForJoystick(const QVector<float>& wrench);
    void controllerFeedback(const QVector<float>& pose,
                            const QVector<float>& twist);
    void updateScalingFactor(float offset);

    void logMasterTwist(const QVector<float>& twist);
    void logABS(const QVector<float>& pose);
    void logREL(const QVector<float>& pose);
    void logEUL(const QVector<float>& twist);
    void logSMA(const QVector<float>& twist);
    void logWMA(const QVector<float>& twist);
    void logSMM(const QVector<float>& twist);
    void logBLP(const QVector<float>& twist);
    void logSMMBLP(const QVector<float>& twist);

  public slots:
    void onJoystickRequest(bool buttonDown, bool buttonUp,
                           const QMatrix4x4& pose);
    void onControllerFeedback(const QVector<float>& pose,
                              const QVector<float>& twist);
};

}  // namespace teleop


#endif  // SUPERVISOR_H
