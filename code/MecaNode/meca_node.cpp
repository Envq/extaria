#include "meca_node.h"
#include "logs.h"

#include <QDebug>

Q_LOGGING_CATEGORY(logMecaNode, "MecaNode")


// ==========================================================================
teleop::MecaWorker::MecaWorker() {
    qDebug(logMecaNode()) << QThread::currentThreadId()
                          << " | MecaWorker::MecaWorker";
    _meca      = new mecademic::MecaAdapter(this);
    _loopTimer = new QTimer(this);
    _loopTimer->setTimerType(Qt::TimerType::PreciseTimer);

    connect(this, &MecaWorker::finished, _loopTimer, &QTimer::stop);
    connect(_loopTimer, &QTimer::timeout, this, &MecaWorker::dutyCycle,
            Qt::DirectConnection);
    connect(_meca, &mecademic::MecaAdapter::abort, this, &MecaWorker::finished,
            Qt::DirectConnection);
    connect(_meca, &mecademic::MecaAdapter::feedback, this,
            &MecaWorker::onFeedback, Qt::QueuedConnection);

    auto& settings         = SettingsManager::getInstance();
    _loopPeriod            = settings.getUnsigned("meca/period");
    _robotIP               = settings.getQString("meca/ip");
    _blending              = settings.getFloat("meca/blending");
    _jointVelocity         = settings.getFloat("meca/jointVelocity");
    _jointAcceleration     = settings.getFloat("meca/jointAcceleration");
    _velocityTimeout       = settings.getFloat("meca/velocityTimeout");
    _cartesianAcceleration = settings.getFloat("meca/cartesianAcceleration");
    _tcp                   = settings.getQVector("task/fla_T_tcp");
    _origin                = settings.getQVector("task/wsl_T_ori");
    _logEnabled            = settings.getBool("nodes/enable_logging_slave");

    if (_logEnabled) {
        const unsigned log_size = settings.getUnsigned("nodes/log_size");
        // REQUEST POSE ABSOLUTE
        auto req_poseAbs = new Logger("request_poseAbs", log_size, this);
        connect(this, &MecaWorker::logRequestPoseAbs, req_poseAbs,
                &Logger::write, Qt::DirectConnection);
        // REQUEST POSE RELATIVE
        auto req_poseRel = new Logger("request_poseRel", log_size, this);
        connect(this, &MecaWorker::logRequestPoseRel, req_poseRel,
                &Logger::write, Qt::DirectConnection);
        // REQUEST TWIST
        auto req_twist = new Logger("request_twist", log_size, this);
        connect(this, &MecaWorker::logRequestTwist, req_twist, &Logger::write,
                Qt::DirectConnection);
        // CURRENT POSE
        auto cur_pose = new Logger("current_pose", log_size, this);
        connect(this, &MecaWorker::logCurrentPose, cur_pose, &Logger::write,
                Qt::DirectConnection);
        // CURRENT TWIST
        auto cur_twist = new Logger("current_twist", log_size, this);
        connect(this, &MecaWorker::logCurrentTwist, cur_twist, &Logger::write,
                Qt::DirectConnection);
    }
}


teleop::MecaWorker::~MecaWorker() {
    qDebug(logMecaNode()) << QThread::currentThreadId()
                          << " | MecaWorker::~MecaWorker";
}


void teleop::MecaWorker::setRequest(const teleop::MecaRequestData& request) {
    _mutex.lock();
    _request = request;
    _mutex.unlock();
}


void teleop::MecaWorker::onStart() {
    qDebug(logMecaNode()) << QThread::currentThreadId()
                          << " | MecaWorker::onStart";
    _meca->initCommunication(_robotIP);
    _loopTimer->start(_loopPeriod);
}


void teleop::MecaWorker::dutyCycle() {
    switch (_state) {
        case MecaState::Init: {
            qInfo(logMecaNode()) << "MecaNode State: Init";
            _meca->reset();
            _meca->activateRobot();
            _meca->homing();
            _state = MecaState::WaitForInit;
            qInfo(logMecaNode()) << "MecaNode State: WaitForInit";
            break;
        }
        case MecaState::WaitForInit: {
            if (_meca->isActivated() && _meca->isHomed()) {
                _meca->setTCP(_tcp);
                _meca->setMonitoringInterval(0.001);  // 0.001, 0.015, 1
                _state = MecaState::Start;
                qInfo(logMecaNode())
                    << "Meca Status: " << _meca->getRobotStatus();
                qInfo(logMecaNode()) << "MecaNode State: Start";
            }
            break;
        }
        case MecaState::Start: {
            _meca->setJointVel(10);
            _meca->setJointAcc(50);
            _meca->movePose(_origin);
            _state = MecaState::WaitForStart;
            qInfo(logMecaNode()) << "MecaNode State: WaitForStart";
            break;
        }
        case MecaState::WaitForStart: {
            if (_meca->isEndOfBlockReached()) {
                _meca->setBlending(_blending);
                _meca->setVelTimeout(_velocityTimeout);
                _meca->setCartLinVel(150);  // 0.001, 150, 1000
                _meca->setCartAngVel(45);   // 0.001,  45,  300
                _meca->setCartAcc(_cartesianAcceleration);
                _meca->setJointVel(_jointVelocity);
                _meca->setJointAcc(_jointAcceleration);
                _state = MecaState::Teleop;
                qInfo(logMecaNode()) << "MecaNode State: Teleop";
            }
            break;
        }
        case MecaState::Teleop: {
            MecaRequestData req;
            _mutex.lock();
            req            = _request;
            _request.fired = true;
            _mutex.unlock();

            if (!req.fired) {
                switch (req.mode) {
                    case Mode::vel:
                        _meca->moveTwist(req.twist);
                        break;
                    case Mode::rel:
                        _meca->movePose(req.poseRel);
                        break;
                    case Mode::abs:
                        _meca->movePose(req.poseAbs);
                        break;
                }
                if (_logEnabled) {
                    emit logRequestTwist(req.twist);
                    emit logRequestPoseRel(req.poseRel);
                    emit logRequestPoseAbs(req.poseAbs);
                }
            }
            break;
        }
    }
}


void teleop::MecaWorker::onFeedback(const QVector<float>& pose,
                                    const QVector<float>& twist) {
    emit feedback(pose, twist);
    if (_logEnabled) {
        emit logCurrentPose(pose);
        emit logCurrentTwist(twist);
    }
}


// ==========================================================================
teleop::MecaNode::MecaNode(QObject* parent) : QObject(parent) {
    qDebug(logMecaNode()) << QThread::currentThreadId()
                          << " | MecaNode::MecaNode";
    _thread = new QThread(this);
    _worker = new MecaWorker();

    connect(_thread, &QThread::started, _worker, &MecaWorker::onStart);
    connect(_worker, &MecaWorker::feedback, this, &MecaNode::feedback,
            Qt::DirectConnection);
    connect(_worker, &MecaWorker::finished, this, &MecaNode::finished);
    connect(this, &MecaNode::finished, _thread, &QThread::quit);
    connect(_thread, &QThread::finished, _worker, &MecaWorker::deleteLater);
}


teleop::MecaNode::~MecaNode() {
    qDebug(logMecaNode()) << QThread::currentThreadId()
                          << " | MecaNode::~MecaNode";
    _thread->quit();
    _thread->wait();
}


void teleop::MecaNode::onStart() {
    qDebug(logMecaNode()) << QThread::currentThreadId() << " | MecaNode::start";
    _worker->moveToThread(_thread);
    _thread->start(QThread::TimeCriticalPriority);
}


void teleop::MecaNode::onRequest(const QVector<float>& poseAbs,
                                 const QVector<float>& poseRel,
                                 const QVector<float>& twist,
                                 const teleop::Mode&   mode) {
    MecaRequestData data;
    data.fired   = false;
    data.poseAbs = poseAbs;
    data.poseRel = poseRel;
    data.twist   = twist;
    data.mode    = mode;
    _worker->setRequest(data);
}
