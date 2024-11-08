#include "generators.h"
#include "kinematic.h"
#include "logs.h"
#include "settings.h"

#include <QDebug>
#include <QThread>
#include <QtMath>

Q_LOGGING_CATEGORY(logGenerators, "Generators")

namespace {
QMatrix4x4 _adj;
}

// ==========================================================================
teleop::MotionGenerator::MotionGenerator(QObject* parent) : QObject(parent) {
    qDebug(logGenerators())
        << QThread::currentThreadId() << " | MotionGenerator created";
    // get infos
    auto& settings    = SettingsManager::getInstance();
    _scalingFactor    = settings.getFloat("task/scaling_factor_start");
    _newPoseThreshold = settings.getFloat("task/new_pose_threshold");
    _relativeMode     = settings.getRelativeMode("task/relative_mode");
    // all
    _wsl_T_ori = poseXYZ_to_matrix(settings.getQVector("task/wsl_T_ori"));
    _ori_T_wma = poseXYZ_to_matrix(settings.getQVector("task/ori_T_wma"));
    _wsl_T_wma = _wsl_T_ori * _ori_T_wma;
    _hip_T_pen = poseXYZ_to_matrix(settings.getQVector("task/hip_T_pen"));
    _pen_T_tcp = _ori_T_wma.inverted();
    _hip_T_tcp = _hip_T_pen * _pen_T_tcp;
    // relative
    _wsl_T_cur = _wsl_T_ori;
    switch (_relativeMode) {
        case RelativeMode::fix: {
            break;
        }
        case RelativeMode::drg: {
            _reallign = poseXYZ_to_matrix({0, 0, 0, 0, 0, 0});
            break;
        }
        case RelativeMode::var: {
            break;
        }
    }
    // other
}


void teleop::MotionGenerator::update(const QVector<float>& pose) {
    if (_firstTime) {
        _timer.start();
        _firstTime         = false;
        _currPose          = pose;
        _lastPose          = pose;
        _lastPosePerformed = _currPose;
    } else {
        _lastPose = _currPose;
        _currPose = pose;
    }
}


void teleop::MotionGenerator::update(const QMatrix4x4& wma_T_hip,
                                     bool              restart) {
    _wma_T_hip = wma_T_hip;
    _wma_T_hip(0, 3) *= _scalingFactor;
    _wma_T_hip(1, 3) *= _scalingFactor;
    _wma_T_hip(2, 3) *= _scalingFactor;
    auto wsl_T_tcp = _wsl_T_wma * _wma_T_hip * _hip_T_tcp;
    //    auto wsl_T_tcp = _wsl_T_ori * _ori_T_wma * _wma_T_hip * _hip_T_pen *
    //                     _ori_T_wma.inverted();
    if (restart || _firstTime) {
        _timer.start();
        _firstTime = false;
        // twist
        _currPose = matrix_to_poseXYZ(wsl_T_tcp);
        _lastPose = _currPose;
        // pose optimization
        _lastPosePerformed = _currPose;
        // relative
        switch (_relativeMode) {
            case RelativeMode::fix: {
                _ref_T_wma = (_wma_T_hip * _hip_T_tcp).inverted();
                break;
            }
            case RelativeMode::drg: {
                _ref_T_wma = (_wma_T_hip * _hip_T_tcp * _reallign).inverted();
                break;
            }
            case RelativeMode::var: {
                _hip_T_adj       = _wma_T_hip.inverted();
                _hip_T_adj(0, 3) = 0;
                _hip_T_adj(1, 3) = 0;
                _hip_T_adj(2, 3) = 0;
                _ref_T_wma = (_wma_T_hip * _hip_T_adj * _pen_T_tcp * _reallign)
                                 .inverted();
                break;
            }
        }
        _adj       = _wma_T_hip * _hip_T_pen;
        _adj(0, 3) = 0;
        _adj(1, 3) = 0;
        _adj(2, 3) = 0;
    } else {
        _lastPose = _currPose;
        _currPose = matrix_to_poseXYZ(wsl_T_tcp);
    }
}


void teleop::MotionGenerator::reIndexing() {
    _wsl_T_cur = _wsl_T_tcp;
    if (_relativeMode == RelativeMode::drg ||
        _relativeMode == RelativeMode::var) {
        _reallign       = _wsl_T_tcp;
        _reallign(0, 3) = 0;
        _reallign(1, 3) = 0;
        _reallign(2, 3) = 0;
    }
}


QVector<float> teleop::MotionGenerator::getRelativePose() {
    switch (_relativeMode) {
        case RelativeMode::fix: {
            _wsl_T_tcp = _wsl_T_cur * _ref_T_wma * _wma_T_hip * _hip_T_tcp;
            return matrix_to_poseXYZ(_wsl_T_tcp);
        }
        case RelativeMode::drg: {
            _wsl_T_tcp =
                _wsl_T_cur * _ref_T_wma * _wma_T_hip * _hip_T_tcp * _reallign;
            return matrix_to_poseXYZ(_wsl_T_tcp);
        }
        case RelativeMode::var: {
            _wsl_T_tcp = _wsl_T_cur * _ref_T_wma * _wma_T_hip * _hip_T_adj *
                         _pen_T_tcp * _reallign;
            return matrix_to_poseXYZ(_wsl_T_tcp);
        }
    }
}


QVector<float> teleop::MotionGenerator::getAbsolutePose() {
    return _currPose;
}


QVector<float> teleop::MotionGenerator::getTwist() {
    float time = _timer.nsecsElapsed() * 1e-9;  // [sec]
    _timer.start();
    if (time == 0) {
        qWarning() << "Try to generate twist with time elapsed == 0...";
    }
    // clang-format off
    return{(_currPose[0] - _lastPose[0]) / time,
           (_currPose[1] - _lastPose[1]) / time,
           (_currPose[2] - _lastPose[2]) / time,
           (_currPose[3] - _lastPose[3]) / time,
           (_currPose[4] - _lastPose[4]) / time,
           (_currPose[5] - _lastPose[5]) / time};
    // clang-format on
}


bool teleop::MotionGenerator::isEnoughDistant() {
    if (_getPoseDistanceFromLast() > _newPoseThreshold) {
        _lastPosePerformed = _currPose;
        return true;
    }
    return false;
}


float teleop::MotionGenerator::_getPoseDistanceFromLast() {
    return qSqrt(qPow((_currPose[0] - _lastPosePerformed[0]), 2) +
                 qPow((_currPose[1] - _lastPosePerformed[1]), 2) +
                 qPow((_currPose[2] - _lastPosePerformed[2]), 2) +
                 qPow((_currPose[3] - _lastPosePerformed[3]), 2) +
                 qPow((_currPose[4] - _lastPosePerformed[4]), 2) +
                 qPow((_currPose[5] - _lastPosePerformed[5]), 2));
}


void teleop::MotionGenerator::onUpdateScalingFactor(float offset) {
    _scalingFactor += offset;
}


// ==========================================================================
teleop::ForceGenerator::ForceGenerator(QObject* parent) : QObject(parent) {
    auto& settings = SettingsManager::getInstance();
    _stiffness     = settings.getFloat("task/feedback_stiffness");
    _forceLimit    = settings.getFloat("task/feedback_max_force");
    auto wsl_T_ori = settings.getQVector("task/wsl_T_ori");
    _origin        = {wsl_T_ori[0], wsl_T_ori[1], wsl_T_ori[2]};
    _wma_T_ori =
        poseXYZ_to_matrix(settings.getQVector("task/ori_T_wma")).inverted();
}


QVector<float>
teleop::ForceGenerator::_reMapping(const QVector<float>& wrench) {
    auto result =
        matrix_to_poseXYZ(_adj * _wma_T_ori * poseXYZ_to_matrix(wrench));

    for (int i = 0; i < 3; ++i) {
        if (result[i] < -_forceLimit) {
            qWarning(logGenerators()).nospace()
                << "time=" << LogClock::getInstance().getMilliseconds()
                << " wrench[" << i << "]=" << result[i]
                << " is minor of the force limit:" << -_forceLimit;
            //            result[i] = 0;
            result[i] = -_forceLimit;
        }
        if (result[i] > _forceLimit) {
            qWarning(logGenerators()).nospace()
                << "time=" << LogClock::getInstance().getMilliseconds()
                << " wrench[" << i << "]=" << result[i]
                << " is major of the force limit:" << _forceLimit;
            //            result[i] = 0;
            result[i] = _forceLimit;
        }
    }
    for (int i = 3; i < 6; ++i) {
        result[i] = 0;
    }
    return result;
}


QVector<float>
teleop::ForceGenerator::_getVecDifference(const QVector<float>& a,
                                          const QVector<float>& b) {
    return {(a[0] - b[0]), (a[1] - b[1]), (a[2] - b[2])};
}


QVector<float> teleop::ForceGenerator::_getVecScaling(const QVector<float>& a,
                                                      float factor) {
    auto v = a;
    for (int i = 0; i < v.size(); ++i) {
        v[i] *= factor;
        if (qIsNaN(v[i])) {
            v[i] = 0;
        }
    }
    return v;
}


float teleop::ForceGenerator::_getVecMagnitude(const QVector<float>& a) {
    return qSqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}


QVector<float>
teleop::ForceGenerator::_getVecDirection(const QVector<float>& a,
                                         const QVector<float>& b) {
    auto  v = _getVecDifference(a, b);
    float m = _getVecMagnitude(v);
    return _getVecScaling(v, 1 / m);
}


// ==========================================================================
teleop::SphereForceGenerator::SphereForceGenerator(QObject* parent)
    : ForceGenerator(parent) {
    qDebug(logGenerators())
        << QThread::currentThreadId() << " | SphereForceGenerator created";
    _origin[0] += _offset[0];
    _origin[1] += _offset[1];
    _origin[2] += _offset[2];
}


QVector<float>
teleop::SphereForceGenerator::getForceFrom(const QVector<float>& pose) {
    QVector<float> difference  = _getVecDifference(pose, _origin);
    double         distance    = _getVecMagnitude(difference);
    double         penetration = _radius - distance;

    if (penetration <= 0) {
        return {0, 0, 0, 0, 0, 0};
    }
    // f = k * x = stiffness * (penetration * versor)
    float x = _stiffness * penetration * (difference[0] / distance);
    float y = _stiffness * penetration * (difference[1] / distance);
    float z = _stiffness * penetration * (difference[2] / distance);

    return _reMapping({x, y, z, 0, 0, 0});
}


// ==========================================================================
teleop::AnchorForceGenerator::AnchorForceGenerator(QObject* parent)
    : ForceGenerator(parent) {
    qDebug(logGenerators())
        << QThread::currentThreadId() << " | AnchorForceGenerator created";
    _origin[0] += _offset[0];
    _origin[1] += _offset[1];
    _origin[2] += _offset[2];
}


QVector<float>
teleop::AnchorForceGenerator::getForceFrom(const QVector<float>& pose) {
    QVector<float> difference = _getVecDifference(_origin, pose);

    float x = _stiffness * difference[0];
    float y = _stiffness * difference[1];
    float z = _stiffness * difference[2];

    return _reMapping({x, y, z, 0, 0, 0});
}


// ==========================================================================
teleop::LinearForceGenerator::LinearForceGenerator(QObject* parent)
    : ForceGenerator(parent) {
    qDebug(logGenerators())
        << QThread::currentThreadId() << " | LinearForceGenerator created";
}


QVector<float>
teleop::LinearForceGenerator::getForceFrom(const QVector<float>& pose) {
    float x = 0;
    float y = _stiffness * (_origin[1] - pose[1]);
    float z = _stiffness * (_origin[2] - pose[2]);

    //    qDebug() << LogClock::getInstance().getMilliseconds() << _origin[2]
    //             << pose[2] << _origin[2] - pose[2] << z;

    //    qDebug() << x << y << z;

    return _reMapping({x, y, z, 0, 0, 0});
}


// ==========================================================================
teleop::TriangleForceGenerator::TriangleForceGenerator(QObject* parent)
    : ForceGenerator(parent) {
    qDebug(logGenerators())
        << QThread::currentThreadId() << " | TriangleForceGenerator created";

    auto& settings = SettingsManager::getInstance();
    _direction     = settings.getBool("task/feedback_tri_dir_pos") ? 1.0 : -1.0;
    _offset        = settings.getFloat("task/feedback_tri_offset");
    _pathlen       = settings.getFloat("task/feedbakc_tri_len");
    _center        = _pathlen / 2.0;
}


QVector<float>
teleop::TriangleForceGenerator::getForceFrom(const QVector<float>& pose) {
    float progress = (pose[0] - _origin[0]) - _offset * _direction;
    float goal     = _process(progress * _direction);

    float x = 0;
    float y = _stiffness * (goal - pose[1]);
    float z = _stiffness * (_origin[2] - pose[2]);

    return _reMapping({x, y, z, 0, 0, 0});
}


float teleop::TriangleForceGenerator::_process(float progress) {
    float adjust = 0.0;
    if (progress > 0 && progress < _pathlen) {
        if (progress < _center) {
            adjust = progress;
        } else {
            adjust = _pathlen - progress;
        }
    }
    return _origin[1] + adjust;
}


// ==========================================================================
teleop::OpponentForceGenerator::OpponentForceGenerator(QObject* parent)
    : ForceGenerator(parent) {
    qDebug(logGenerators())
        << QThread::currentThreadId() << " | OpponentForceGenerator created";
}


QVector<float>
teleop::OpponentForceGenerator::getForceFrom(const QVector<float>& pose) {
    if (_firstTime) {
        _firstTime = false;
        return {0, 0, 0, 0, 0, 0};
    }
    auto  diff      = _getVecDifference(pose, _lastPose);
    float magnitude = _getVecMagnitude(diff);
    auto  versor    = _getVecScaling(diff, 1 / magnitude);
    _lastPose       = pose;

    float req = 0;
    if (magnitude > _magnitudeThreshold) {
        if (versor[1] > 0) {
            req = -_force;
        }
        if (versor[1] < 0) {
            req = _force;
        }
    }

    if (req != _current) {
        _counter++;
        if (_counter > _counterThreshold) {
            _current = req;
        }
    } else {
        _counter = 0;
    }

    float x = 0;
    float y = _current;
    float z = 0;

    qDebug(logGenerators()) << x << y << z;
    return _reMapping({x, y, z, 0, 0, 0});
}
