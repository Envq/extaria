#ifndef GENERATORS_H
#define GENERATORS_H

#include "settings.h"

#include <QElapsedTimer>
#include <QLoggingCategory>
#include <QMatrix4x4>
#include <QObject>
#include <QVector>

Q_DECLARE_LOGGING_CATEGORY(logGenerators)


namespace teleop {

// ==========================================================================
class MotionGenerator : public QObject {
    Q_OBJECT

  public:
    MotionGenerator(QObject* parent = nullptr);
    MotionGenerator(const MotionGenerator&) = delete;
    MotionGenerator(MotionGenerator&&)      = delete;

    void           update(const QMatrix4x4& wma_T_hip, bool restart);  // touch
    void           reIndexing();
    QVector<float> getRelativePose();
    QVector<float> getAbsolutePose();
    QVector<float> getTwist();
    bool           isEnoughDistant();

  private:
    bool          _firstTime     = true;
    float         _scalingFactor = 1.0;
    RelativeMode  _relativeMode  = RelativeMode::fix;
    QElapsedTimer _timer;
    // all
    QMatrix4x4 _wsl_T_ori;
    QMatrix4x4 _ori_T_wma;
    QMatrix4x4 _wsl_T_wma;
    QMatrix4x4 _hip_T_pen;
    QMatrix4x4 _pen_T_tcp;
    QMatrix4x4 _hip_T_tcp;
    // relative
    QMatrix4x4 _wsl_T_cur;
    QMatrix4x4 _reallign;
    QMatrix4x4 _wma_T_hip;
    QMatrix4x4 _ref_T_wma;
    QMatrix4x4 _hip_T_adj;
    // for Twist mode
    QVector<float> _currPose;
    QVector<float> _lastPose;
    // for Pose optimization
    float          _newPoseThreshold;
    QVector<float> _lastPosePerformed;
    // for reindexing
    QMatrix4x4 _wsl_T_tcp;

    float _getPoseDistanceFromLast();

  public slots:
    void onUpdateScalingFactor(float offset);
};

// ==========================================================================
class ForceGenerator : public QObject {
    Q_OBJECT

  public:
    ForceGenerator(QObject* parent = nullptr);
    ForceGenerator(const ForceGenerator&) = delete;
    ForceGenerator(ForceGenerator&&)      = delete;

    virtual QVector<float> getForceFrom(const QVector<float>& pose) = 0;

  protected:
    float          _stiffness  = 0.25;
    float          _forceLimit = 1.0f;
    QVector<float> _origin     = {0, 0, 0};
    QMatrix4x4     _wma_T_ori;

    QVector<float> _reMapping(const QVector<float>& wrench);
    QVector<float> _getVecDifference(const QVector<float>& a,
                                     const QVector<float>& b);
    QVector<float> _getVecScaling(const QVector<float>& a, float factor);
    float          _getVecMagnitude(const QVector<float>& a);
    QVector<float> _getVecDirection(const QVector<float>& a,
                                    const QVector<float>& b);
};

// ==========================================================================
class SphereForceGenerator : public ForceGenerator {
    Q_OBJECT

  public:
    SphereForceGenerator(QObject* parent = nullptr);
    SphereForceGenerator(const SphereForceGenerator&) = delete;
    SphereForceGenerator(SphereForceGenerator&&)      = delete;

    QVector<float> getForceFrom(const QVector<float>& pose) override;

  private:
    QVector<float> _offset = {0, 0, -100};
    float          _radius = 40.0;
};

// ==========================================================================
class AnchorForceGenerator : public ForceGenerator {
    Q_OBJECT

  public:
    AnchorForceGenerator(QObject* parent = nullptr);
    AnchorForceGenerator(const AnchorForceGenerator&) = delete;
    AnchorForceGenerator(AnchorForceGenerator&&)      = delete;

    QVector<float> getForceFrom(const QVector<float>& pose) override;

  private:
    QVector<float> _offset = {0, 0, 0};
};

// ==========================================================================
class LinearForceGenerator : public ForceGenerator {
    Q_OBJECT

  public:
    LinearForceGenerator(QObject* parent = nullptr);
    LinearForceGenerator(const LinearForceGenerator&) = delete;
    LinearForceGenerator(LinearForceGenerator&&)      = delete;

    QVector<float> getForceFrom(const QVector<float>& pose) override;
};

// ==========================================================================
class TriangleForceGenerator : public ForceGenerator {
    Q_OBJECT

  public:
    TriangleForceGenerator(QObject* parent = nullptr);
    TriangleForceGenerator(const TriangleForceGenerator&) = delete;
    TriangleForceGenerator(TriangleForceGenerator&&)      = delete;

    QVector<float> getForceFrom(const QVector<float>& pose) override;

  private:
    float _offset    = 0;
    float _pathlen   = 0;
    float _center    = 0;
    float _direction = 1.0;

    float _process(float progress);
};

// ==========================================================================
class OpponentForceGenerator : public ForceGenerator {
    Q_OBJECT

  public:
    OpponentForceGenerator(QObject* parent = nullptr);
    OpponentForceGenerator(const OpponentForceGenerator&) = delete;
    OpponentForceGenerator(OpponentForceGenerator&&)      = delete;

    QVector<float> getForceFrom(const QVector<float>& pose) override;

  private:
    bool           _firstTime = true;
    QVector<float> _lastPose  = {0, 0, 0, 0, 0, 0};

    float _force              = 0.6;
    float _current            = 0;
    float _magnitudeThreshold = 0.05;
    int   _counter            = 0;
    int   _counterThreshold   = 50;
};

}  // namespace teleop


#endif  // GENERATORS_H
