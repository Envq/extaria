#include "supervisor.h"
#include "logs.h"

#include <QThread>

Q_LOGGING_CATEGORY(logSupervisor, "Supervisor")


// ==========================================================================
teleop::Supervisor::Supervisor(QObject* parent) : QObject(parent) {
    qDebug(logSupervisor())
        << QThread::currentThreadId() << " | Supervisor::Supervisor";
    // LOG CLOCK
    connect(this, &Supervisor::started, this,
            []() { LogClock::getInstance().start(); });

    // INITIALIZATION
    auto& settings        = SettingsManager::getInstance();
    _feedbackFromRobot    = settings.getBool("task/feedback_from_robot");
    _enableLoggingFilters = settings.getBool("nodes/enable_logging_filters");
    _logSize              = settings.getUnsigned("nodes/log_size");
    _taskMode             = settings.getMode("task/mode");
    _feedbackType         = settings.getFeedbackType("task/feedback_type");
    _filterType           = settings.getFilterType("task/twist_filter_type");

    qInfo(logSupervisor()) << "Task mode:    "
                           << convertModeToQString(_taskMode);
    qInfo(logSupervisor()) << "Feedback Type:"
                           << convertFeedbackTypeToQString(_feedbackType);
    qInfo(logSupervisor()) << "Twist Filter: "
                           << convertFilterTypeToQString(_filterType);

    // MOTION GENERATOR
    _motionGenerator = new MotionGenerator(this);

    // FORCE GENERATOR
    switch (_feedbackType) {
        case FeedbackType::none:
            break;
        case FeedbackType::sphere:
            _forceGenerator = new SphereForceGenerator(this);
            break;
        case FeedbackType::anchor:
            _forceGenerator = new AnchorForceGenerator(this);
            break;
        case FeedbackType::linear:
            _forceGenerator = new LinearForceGenerator(this);
            break;
        case FeedbackType::triangle:
            _forceGenerator = new TriangleForceGenerator(this);
            break;
        case FeedbackType::opponent:
            _forceGenerator = new OpponentForceGenerator(this);
            break;
    }

    // FILTERS
    _sma = new SimpleMovingAverage(settings.getFloat("filters/sma"), this);
    _wma = new WeightedMovingAverage(settings.getFloat("filters/wma"), this);
    _smm = new SimpleMovingMedian(settings.getFloat("filters/smm"), this);
    _blp = new ButterworthLowPass(settings.getQVector("filters/blp"), this);

    auto log_master_twist = new Logger("master_twist", _logSize, this);
    connect(this, &Supervisor::logMasterTwist, log_master_twist, &Logger::write,
            Qt::DirectConnection);

    // LOGGING
    if (_enableLoggingFilters) {
        auto log_abs = new Logger("filter_abs", _logSize, this);
        connect(this, &Supervisor::logABS, log_abs, &Logger::write,
                Qt::DirectConnection);
        auto log_rel = new Logger("filter_rel", _logSize, this);
        connect(this, &Supervisor::logREL, log_rel, &Logger::write,
                Qt::DirectConnection);
        auto log_eul = new Logger("filter_eul", _logSize, this);
        connect(this, &Supervisor::logEUL, log_eul, &Logger::write,
                Qt::DirectConnection);
        auto log_sma = new Logger("filter_sma", _logSize, this);
        connect(this, &Supervisor::logSMA, log_sma, &Logger::write,
                Qt::DirectConnection);
        auto log_wma = new Logger("filter_wma", _logSize, this);
        connect(this, &Supervisor::logWMA, log_wma, &Logger::write,
                Qt::DirectConnection);
        auto log_smm = new Logger("filter_smm", _logSize, this);
        connect(this, &Supervisor::logSMM, log_smm, &Logger::write,
                Qt::DirectConnection);
        auto log_blp = new Logger("filter_blp", _logSize, this);
        connect(this, &Supervisor::logBLP, log_blp, &Logger::write,
                Qt::DirectConnection);
        auto log_smmblp = new Logger("filter_smmblp", _logSize, this);
        connect(this, &Supervisor::logSMMBLP, log_smmblp, &Logger::write,
                Qt::DirectConnection);
    }

    // TOUCH JOYSTICK
    if (settings.getBool("nodes/enable_joystick")) {
        qInfo(logSupervisor()) << "Activated: TouchNode";
        auto joystick = new TouchNode(this);
        connect(this, &Supervisor::started, joystick, &TouchNode::onStart);
        connect(joystick, &TouchNode::finished, this, &Supervisor::finished,
                Qt::DirectConnection);
        connect(joystick, &TouchNode::request, this,
                &Supervisor::onJoystickRequest, Qt::QueuedConnection);
        connect(this, &Supervisor::feedbackForJoystick, joystick,
                &TouchNode::onFeedback, Qt::QueuedConnection);
        if (!_feedbackFromRobot) {
            connect(this, &Supervisor::controllerFeedback, this,
                    &Supervisor::onControllerFeedback, Qt::QueuedConnection);
        }
    }

    // ROBOT
    if (settings.getBool("nodes/enable_robot")) {
        qInfo(logSupervisor()) << "Activated: MecaNode";
        auto robot = new MecaNode(this);
        connect(this, &Supervisor::started, robot, &MecaNode::onStart);
        connect(robot, &MecaNode::finished, this, &Supervisor::finished,
                Qt::DirectConnection);
        connect(this, &Supervisor::requestForRobot, robot, &MecaNode::onRequest,
                Qt::QueuedConnection);
        if (_feedbackFromRobot) {
            connect(robot, &MecaNode::feedback, this,
                    &Supervisor::onControllerFeedback, Qt::QueuedConnection);
        }
    }
}


teleop::Supervisor::~Supervisor() {
    qDebug(logSupervisor())
        << QThread::currentThreadId() << " | Supervisor::~Supervisor";
}


void teleop::Supervisor::start() {
    qDebug(logSupervisor())
        << QThread::currentThreadId() << " | Supervisor::start";
    emit started();
}


void teleop::Supervisor::onJoystickRequest(bool buttonDown, bool buttonUp,
                                           const QMatrix4x4& wm_T_hip) {
    // get action
    const bool quit       = !buttonUp && buttonDown;
    const bool perform    = buttonUp;
    const bool newRun     = !_lastPerform && perform;
    const bool reIndexing = _lastPerform && !perform;
    _performFeedback      = false;

    if (quit) {
        emit finished();
    }

    if (perform) {
        _performFeedback = true;
        _motionGenerator->update(wm_T_hip, newRun);
        auto           pose_absolute = _motionGenerator->getAbsolutePose();
        auto           pose_relative = _motionGenerator->getRelativePose();
        auto           raw_twist     = _motionGenerator->getTwist();
        QVector<float> twist         = {0, 0, 0, 0, 0, 0};
        switch (_filterType) {
            case FilterType::none:
                twist = raw_twist;
                break;
            case FilterType::sma:
                twist = _sma->applyFilter(raw_twist);
                break;
            case FilterType::wma:
                twist = _wma->applyFilter(raw_twist);
                break;
            case FilterType::smm:
                twist = _smm->applyFilter(raw_twist);
                break;
            case FilterType::blp:
                twist = _blp->applyFilter(raw_twist);
                break;
            case FilterType::smmblp:
                twist = _blp->applyFilter(_smm->applyFilter(raw_twist));
                break;
        }
        emit logMasterTwist(twist);

        if (_taskMode == Mode::vel || _motionGenerator->isEnoughDistant()) {
            emit requestForRobot(pose_absolute, pose_relative, twist,
                                 _taskMode);
            if (_enableLoggingFilters) {
                emit logABS(pose_absolute);
                emit logREL(pose_relative);
                emit logEUL(raw_twist);
                emit logSMA(_sma->applyFilter(raw_twist));
                emit logWMA(_sma->applyFilter(raw_twist));
                emit logSMM(_smm->applyFilter(raw_twist));
                emit logBLP(_blp->applyFilter(raw_twist));
                emit logSMMBLP(_blp->applyFilter(_smm->applyFilter(raw_twist)));
            }
        }
        emit controllerFeedback(pose_relative, twist);

        //        qDebug(logSupervisor()) << "poseAbs:" << pose_absolute;
        //        qDebug(logSupervisor()) << "PoseRel:" << pose_relative;
        //        qDebug(logSupervisor()) << "Twist:  " << twist;
        //        qDebug(logSupervisor()) << "-----------------";
    }
    if (reIndexing) {
        _motionGenerator->reIndexing();
        emit controllerFeedback({}, {});
    }
    _lastPerform = perform;  // update
}


void teleop::Supervisor::onControllerFeedback(const QVector<float>& pose,
                                              const QVector<float>& twist) {
    if (_feedbackType != FeedbackType::none) {
        if (_performFeedback) {
            emit feedbackForJoystick(_forceGenerator->getForceFrom(pose));
        } else {
            emit feedbackForJoystick({0, 0, 0, 0, 0, 0});
        }
    }
}
