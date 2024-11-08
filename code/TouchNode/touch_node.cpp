#include "touch_node.h"
#include "logs.h"
#include "settings.h"

#include <QDebug>

Q_LOGGING_CATEGORY(logTouchNode, "TouchNode")


// ==========================================================================
teleop::TouchWorker::TouchWorker() {
    qDebug(logTouchNode()) << QThread::currentThreadId()
                           << " | TouchWorker::TouchWorker";
    _touch     = new systems3d::TouchAdapter(this);
    _loopTimer = new QTimer(this);
    _loopTimer->setTimerType(Qt::TimerType::PreciseTimer);

    connect(this, &TouchWorker::finished, _loopTimer, &QTimer::stop);
    connect(_loopTimer, &QTimer::timeout, this, &TouchWorker::dutyCycle,
            Qt::DirectConnection);
    connect(_touch, &systems3d::TouchAdapter::abort, this,
            &TouchWorker::finished, Qt::DirectConnection);

    auto& settings   = SettingsManager::getInstance();
    _loopPeriod      = settings.getUnsigned("touch/period");
    _feedbackEnabled = settings.getBool("touch/enable_feedback");
    _logEnabled      = settings.getBool("nodes/enable_logging_wrench");

    if (_logEnabled) {
        const unsigned log_size   = settings.getUnsigned("nodes/log_size");
        auto           log_wrench = new Logger("wrench", log_size, this);
        connect(this, &TouchWorker::logWrench, log_wrench, &Logger::write,
                Qt::DirectConnection);
    }
}


teleop::TouchWorker::~TouchWorker() {
    qDebug(logTouchNode()) << QThread::currentThreadId()
                           << " | TouchWorker::~TouchWorker";
}


void teleop::TouchWorker::onStart() {
    qDebug(logTouchNode()) << QThread::currentThreadId()
                           << " | TouchWorker::start";
    _touch->start();
    _loopTimer->start(_loopPeriod);
}


void teleop::TouchWorker::dutyCycle() {
    // REQUEST
    _touch->updateState();
    bool buttonDown  = _touch->getButtonDown();
    bool buttonUp    = _touch->getButtonUp();
    auto pose_matrix = _touch->getPoseMatrix();
    emit request(buttonDown, buttonUp, pose_matrix);

    // FEEDBACK
    if (_feedbackEnabled) {
        bool           perform;
        QVector<float> wrench = {0, 0, 0, 0, 0, 0};
        _mutex.lock();
        perform         = !_feedback.fired;
        wrench          = _feedback.wrench;
        _feedback.fired = true;
        _mutex.unlock();

        if (perform) {
            _touch->setForce(wrench);
            //            qDebug() << wrench;
            if (_logEnabled) {
                emit logWrench(wrench);
            }
        }
    }
}


void teleop::TouchWorker::setFeedback(const TouchFeedbackData& feedback) {
    _mutex.lock();
    _feedback = feedback;
    _mutex.unlock();
}


// ==========================================================================
teleop::TouchNode::TouchNode(QObject* parent) : QObject(parent) {
    qDebug(logTouchNode()) << QThread::currentThreadId()
                           << " | TouchNode::TouchNode";
    _thread = new QThread(this);
    _worker = new TouchWorker();

    connect(_thread, &QThread::started, _worker, &TouchWorker::onStart);
    connect(_worker, &TouchWorker::request, this, &TouchNode::request,
            Qt::DirectConnection);
    connect(_worker, &TouchWorker::finished, this, &TouchNode::finished);
    connect(this, &TouchNode::finished, _thread, &QThread::quit);
    connect(_thread, &QThread::finished, _worker, &TouchWorker::deleteLater);
}


teleop::TouchNode::~TouchNode() {
    qDebug(logTouchNode()) << QThread::currentThreadId()
                           << " | TouchNode::~TouchNode";
    _thread->quit();
    _thread->wait();
}


void teleop::TouchNode::onStart() {
    qDebug(logTouchNode()) << QThread::currentThreadId()
                           << " | TouchNode::start";
    _worker->moveToThread(_thread);
    _thread->start(QThread::TimeCriticalPriority);
}


void teleop::TouchNode::onFeedback(const QVector<float>& wrench) {
    TouchFeedbackData data;
    data.fired  = false;
    data.wrench = wrench;
    _worker->setFeedback(data);
}
