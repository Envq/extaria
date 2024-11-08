#include "meca_adapter.h"

#include <QDateTime>
#include <QDebug>
#include <QThread>

Q_LOGGING_CATEGORY(logMecaAdapter, "MecaAdapter")


// ==========================================================================
QDebug mecademic::operator<<(QDebug                       debug,
                             const mecademic::MecaStatus& status) {
    QDebugStateSaver saver(debug.nospace());
    debug << "[";
    debug << "actived=" << static_cast<int>(status.activated) << ", ";
    debug << "homed=" << static_cast<int>(status.homed) << ", ";
    debug << "sim=" << static_cast<int>(status.inSimulation) << ", ";
    debug << "err=" << static_cast<int>(status.inError) << ", ";
    debug << "pause=" << static_cast<int>(status.inPauseMotion) << ", ";
    debug << "eob=" << static_cast<int>(status.endOfBlock) << ", ";
    debug << "eom=" << static_cast<int>(status.endOfMovement);
    debug << "]";
    return debug;
}


// ==========================================================================
// --------------------------------------------------------------------------
// CONSTRUCTORS AND DECONSTRUCTORS
mecademic::MecaAdapter::MecaAdapter(QObject* parent) : QObject(parent) {
    // CONTROL SOCKET
    _controlSocket = new QTcpSocket(this);
    connect(_controlSocket, &QTcpSocket::errorOccurred, this,
            &MecaAdapter::_onControlErrorOccurred);
    connect(_controlSocket, &QTcpSocket::readyRead, this,
            &MecaAdapter::_onControlReadyRead);
    // MONITORING SOCKET
    _monitoringSocket = new QTcpSocket(this);
    connect(_monitoringSocket, &QTcpSocket::errorOccurred, this,
            &MecaAdapter::_onMonitoringErrorOccurred);
    connect(_monitoringSocket, &QTcpSocket::readyRead, this,
            &MecaAdapter::_onMonitoringReadyRead);
}


mecademic::MecaAdapter::~MecaAdapter() {
    _controlSocket->close();
    _monitoringSocket->close();
}

// --------------------------------------------------------------------------
// MOTION COMMANDS
void mecademic::MecaAdapter::delay(float sec) {
    _sendCommand(QString("Delay(%0)\n").arg(QString::number(sec)));
}


void mecademic::MecaAdapter::setCheckpoint(int n) {
    //    [3030][n]
    _sendCommand(
        QString("SetCheckpoint(%0)\n").arg(QString::number(_norm(n, 1, 8000))));
}


unsigned mecademic::MecaAdapter::addCheckpoint() {
    _checkpointRequest =
        (_checkpointRequest == 8000) ? 1 : _checkpointRequest + 1;
    setCheckpoint(_checkpointRequest);
    return _checkpointRequest;
}


void mecademic::MecaAdapter::setTCP(const QVector<float>& tcp) {
    _sendCommand(QString("SetTRF(%0,%1,%2,%3,%4,%5)\n")
                     .arg(QString::number(tcp[0]), QString::number(tcp[1]),
                          QString::number(tcp[2]), QString::number(tcp[3]),
                          QString::number(tcp[4]), QString::number(tcp[5])));
}


void mecademic::MecaAdapter::setCartLinVel(float v) {
    _sendCommand(QString("SetCartLinVel(%0)\n")
                     .arg(QString::number(_norm(v, 0.0010, 1000))));
}


void mecademic::MecaAdapter::setCartAngVel(float w) {
    _sendCommand(QString("SetCartAngVel(%0)\n")
                     .arg(QString::number(_norm(w, 0.0010, 300))));
}


void mecademic::MecaAdapter::setCartAcc(float perc) {
    _sendCommand(QString("SetCartAcc(%0)\n")
                     .arg(QString::number(_norm(perc, 0.001, 600))));
}


void mecademic::MecaAdapter::setJointVel(float perc) {
    _sendCommand(QString("SetJointVel(%0)\n")
                     .arg(QString::number(_norm(perc, 0.001, 100))));
}


void mecademic::MecaAdapter::setJointAcc(float perc) {
    _sendCommand(QString("SetJointAcc(%0)\n")
                     .arg(QString::number(_norm(perc, 0.001, 150))));
}


void mecademic::MecaAdapter::setBlending(float perc) {
    _sendCommand(
        QString("SetBlending(%0)\n").arg(QString::number(_norm(perc, 0, 100))));
}


void mecademic::MecaAdapter::setConf(int shoulder, int elbow, int wrist) {
    _sendCommand(QString("SetConf(%0,%1,%2)\n")
                     .arg(QString::number(_normConf(shoulder)),
                          QString::number(_normConf(elbow)),
                          QString::number(_normConf(wrist))));
}


void mecademic::MecaAdapter::setConfTurn(int turn) {
    _sendCommand(QString("setConfTurn(%0)\n")
                     .arg(QString::number(_norm(turn, -100, 100))));
}


void mecademic::MecaAdapter::setAutoConf(bool enable) {
    _sendCommand(QString("SetAutoConf(%0)\n").arg(QString::number(enable)));
}


void mecademic::MecaAdapter::setAutoConfTurn(bool enable) {
    _sendCommand(QString("SetAutoConfTurn(%0)\n").arg(QString::number(enable)));
}


void mecademic::MecaAdapter::moveJoints(const QVector<float>& joints) {
    _sendCommand(QString("MoveJoints(%0,%1,%2,%3,%4,%5)\n")
                     .arg(QString::number(_norm(joints[0], -175, 175)),
                          QString::number(_norm(joints[1], -70, 90)),
                          QString::number(_norm(joints[2], -135, 70)),
                          QString::number(_norm(joints[3], -170, 170)),
                          QString::number(_norm(joints[4], -115, 115)),
                          QString::number(_norm(joints[5], -36000, 36000))));
}


void mecademic::MecaAdapter::moveLin(const QVector<float>& pose) {
    _sendCommand(QString("MoveLin(%0,%1,%2,%3,%4,%5)\n\n")
                     .arg(QString::number(pose[0]), QString::number(pose[1]),
                          QString::number(pose[2]), QString::number(pose[3]),
                          QString::number(pose[4]), QString::number(pose[5])));
}


void mecademic::MecaAdapter::moveLinRel(const QVector<float>& pose) {
    _sendCommand(QString("MoveLinRelTRF(%0,%1,%2,%3,%4,%5)\n")
                     .arg(QString::number(pose[0]), QString::number(pose[1]),
                          QString::number(pose[2]), QString::number(pose[3]),
                          QString::number(pose[4]), QString::number(pose[5])));
}


void mecademic::MecaAdapter::movePose(const QVector<float>& pose) {
    _sendCommand(QString("MovePose(%0,%1,%2,%3,%4,%5)\n")
                     .arg(QString::number(pose[0]), QString::number(pose[1]),
                          QString::number(pose[2]), QString::number(pose[3]),
                          QString::number(pose[4]), QString::number(pose[5])));
}


void mecademic::MecaAdapter::setVelTimeout(float sec) {
    _sendCommand(QString("SetVelTimeout(%0)\n")
                     .arg(QString::number(_norm(sec, 0.001, 1))));
}


void mecademic::MecaAdapter::moveJointVel(const QVector<float>& jointsVel) {
    _sendCommand(QString("MoveJointsVel(%0,%1,%2,%3,%4,%5)\n")
                     .arg(QString::number(_norm(jointsVel[0], -150, 150)),
                          QString::number(_norm(jointsVel[1], -150, 150)),
                          QString::number(_norm(jointsVel[2], -180, 180)),
                          QString::number(_norm(jointsVel[3], -300, 300)),
                          QString::number(_norm(jointsVel[4], -300, 300)),
                          QString::number(_norm(jointsVel[5], -300, 300))));
}


void mecademic::MecaAdapter::moveTwist(const QVector<float>& twist) {
    _sendCommand(QString("MoveLinVelWRF(%0,%1,%2,%3,%4,%5)\n")
                     .arg(QString::number(_norm(twist[0], -1000, 1000)),
                          QString::number(_norm(twist[1], -1000, 1000)),
                          QString::number(_norm(twist[2], -1000, 1000)),
                          QString::number(_norm(twist[3], -300, 300)),
                          QString::number(_norm(twist[4], -300, 300)),
                          QString::number(_norm(twist[5], -300, 300))));
}


void mecademic::MecaAdapter::setGripperVel(float perc) {
    _sendCommand(QString("SetGripperVel(%0)\n")
                     .arg(QString::number(_norm(perc, 5, 100))));
}


void mecademic::MecaAdapter::setGripperForce(float perc) {
    _sendCommand(QString("SetGripperForce(%0)\n")
                     .arg(QString::number(_norm(perc, 5, 100))));
}


void mecademic::MecaAdapter::gripperOpen() {
    _sendCommand(QString("GripperOpen\n"));
}


void mecademic::MecaAdapter::gripperClose() {
    _sendCommand(QString("GripperClose\n"));
}

// --------------------------------------------------------------------------
// REQUEST COMMANDS
void mecademic::MecaAdapter::activateRobot() {
    _sendCommand(QString("ActivateRobot\n"));
}


void mecademic::MecaAdapter::deactivateRobot() {
    _sendCommand(QString("DeactivateRobot\n"));
}


void mecademic::MecaAdapter::activateSim() {
    _sendCommand(QString("ActivateSim\n"));
}


void mecademic::MecaAdapter::deactivateSim() {
    _sendCommand(QString("DeactivateSim\n"));
}


void mecademic::MecaAdapter::homing() {
    _sendCommand(QString("Home\n"));
}


void mecademic::MecaAdapter::resetError() {
    _sendCommand(QString("ResetError\n"));
}


void mecademic::MecaAdapter::pauseMotion() {
    _sendCommand(QString("PauseMotion\n"));
}


void mecademic::MecaAdapter::resumeMotion() {
    _sendCommand(QString("ResumeMotion\n"));
}


void mecademic::MecaAdapter::clearMotion() {
    _sendCommand(QString("ClearMotion\n"));
}


void mecademic::MecaAdapter::getCmdPendingCount() {
    _sendCommand(QString("GetCmdPendingCount\n"));
}


void mecademic::MecaAdapter::setMonitoringInterval(float sec) {
    _sendCommand(QString("SetMonitoringInterval(%0)\n")
                     .arg(QString::number(_norm(sec, 0.001, 1))));
}


void mecademic::MecaAdapter::reset() {
    resetError();
    clearMotion();
    resumeMotion();
}

// --------------------------------------------------------------------------
// CUSTOM COMMANDS
void mecademic::MecaAdapter::initCommunication(const QString& address) {
    bool quit = false;
    // Try to connect to the Control Port
    _controlSocket->connectToHost(address, _controlPort);
    if (!_controlSocket->waitForConnected(500)) {
        qCritical(logMecaAdapter())
            << "Control Port error: " + _controlSocket->errorString();
        quit = true;
    }
    // Try to connect to the Monitoring Port
    _monitoringSocket->connectToHost(address, _monitoringPort);
    if (!_monitoringSocket->waitForConnected(500)) {
        qCritical(logMecaAdapter())
            << "Monitoring Port error: " + _monitoringSocket->errorString();
        quit = true;
    }
    if (quit) {
        emit abort();
    } else {
        _setTime();
        _setRealTimeMonitoring();
    }
}


bool mecademic::MecaAdapter::isActivated() {
    return _activated;
}


bool mecademic::MecaAdapter::isHomed() {
    return _homed;
}


bool mecademic::MecaAdapter::isEndOfBlockReached() {
    auto res           = _endOfBlockReached;
    _endOfBlockReached = false;  // consumed
    return res;
}


QSet<unsigned> mecademic::MecaAdapter::getCheckpointReached() {
    auto res = _checkpointsReached;
    _checkpointsReached.clear();  // consumed
    return res;
}


mecademic::MecaStatus mecademic::MecaAdapter::getRobotStatus() const {
    return _currRobotStatus;
}


QVector<float> mecademic::MecaAdapter::getJoints() const {
    return _currJoints;
}


QVector<float> mecademic::MecaAdapter::getPose() const {
    return _currPose;
}


QVector<float> mecademic::MecaAdapter::getJointVel() const {
    return _currJointVel;
}


QVector<float> mecademic::MecaAdapter::getTwist() const {
    return _currTwist;
}


unsigned mecademic::MecaAdapter::getTime() const {
    return _currTimeStamp;
}

// --------------------------------------------------------------------------
// PRIVATE
void mecademic::MecaAdapter::_setRealTimeMonitoring() {
    _sendCommand(QString("SetRealTimeMonitoring(2210, 2211, 2212, 2214)\n"));
}


void mecademic::MecaAdapter::_setTime() {
    qint64 time = QDateTime::currentDateTime().toSecsSinceEpoch();
    _sendCommand(QString("SetRTC(%0)\n").arg(QString::number(time)));
}


void mecademic::MecaAdapter::_sendCommand(const QString& cmd) {
    _controlSocket->write(cmd.toUtf8());
    if (!_controlSocket->waitForBytesWritten()) {
        qWarning(logMecaAdapter()) << "Error in sending this command: " << cmd;
    }
}


float mecademic::MecaAdapter::_norm(float val, float min, float max) {
    if (val < min) {
        qWarning(logMecaAdapter()) << "normalize " << val << "to " << min;
        return min;
    }
    if (val > max) {
        qWarning(logMecaAdapter()) << "normalize " << val << "to " << max;
        return max;
    }
    return val;
}


int mecademic::MecaAdapter::_normConf(int val) {
    if (val == -1 || val == 1) {
        return val;
    }
    if (val < -1) {
        qWarning(logMecaAdapter()) << "normalize " << val << "to -1";
        return -1;
    }
    qWarning(logMecaAdapter()) << "normalize " << val << "to +1";
    return 1;
}


void mecademic::MecaAdapter::_processControlReply(const QByteArray& reply) {
    unsigned code = reply.mid(1, 4).toUInt();
    QString  msg  = reply.mid(7, reply.size() - 8);
    //    qDebug(logMecaAdapter()) << "[Control   ] " << reply;

    if (code > 3000) {
        switch (code) {
            case 3012: {
                _endOfBlockReached = true;
                break;
            }
            case 3030: {
                _checkpointsReached.insert(msg.toUInt());
                break;
            }
            default: {
                qWarning(logMecaAdapter()) << reply;
                //        emit abort();
            }
        }

    } else if (code < 2000) {
        qWarning(logMecaAdapter()) << "Robot error: " << msg;
        emit abort();

    } else {
        switch (code) {
            case 2000: {
                _activated = true;
                break;
            }
            case 2001: {
                _activated = true;
                break;
            }
            case 2002: {
                _homed = true;
                break;
            }
            case 2003: {
                _homed = true;
                break;
            }
            case 2080: {
                qInfo(logMecaAdapter()) << "pending: " << msg;
                break;
            }
            default: {
                //                qDebug(logMecaAdapter()) << reply;
            }
        }
    }
}


void mecademic::MecaAdapter::_processMonitoringReply(const QByteArray& reply) {
    unsigned code = reply.mid(1, 4).toUInt();
    QString  msg  = reply.mid(7, reply.size() - 8);
    //    qDebug(logMecaAdapter()) << "[Monitoring] " << reply;

    // clang-format off
    /* ERROR FOUND:
    [DEBUG   ][default        ] 2211 "[2211][1110040000,57.5784,-147.1124,153.7645"
    [DEBUG   ][default        ] 2211 "[2211][1154281000,26.5572,-155.5765,164.0912,2.5078,7."
    */
    // clang-format on

    switch (code) {
        case 2030: {
            _currTimeStamp = msg.toUInt();
            break;
        }
        case 2007: {
            auto list = msg.split(",");
            // Update state
            _currRobotStatus.activated     = list[0].toInt();
            _currRobotStatus.homed         = list[1].toInt();
            _currRobotStatus.inSimulation  = list[2].toInt();
            _currRobotStatus.inError       = list[3].toInt();
            _currRobotStatus.inPauseMotion = list[4].toInt();
            _currRobotStatus.endOfBlock    = list[5].toInt();
            _currRobotStatus.endOfMovement = list[6].toInt();
            break;
        }
        case 2210: {
            auto list = msg.split(",");
            if (list.size() < 7) {
                qWarning(logMecaAdapter()) << "size error with 2210" << reply;
            }
            for (int i = 0; i < 6; ++i) {
                _currJoints[i] = list[1 + i].toFloat();
            }
            break;
        }
        case 2211: {
            auto list = msg.split(",");
            if (list.size() < 7) {
                qWarning(logMecaAdapter()) << "size error with 2211" << reply;
            }
            for (int i = 0; i < 6; ++i) {
                _currPose[i] = list[1 + i].toFloat();
            }
            break;
        }
        case 2212: {
            auto list = msg.split(",");
            if (list.size() < 7) {
                qWarning(logMecaAdapter()) << "size error with 2212" << reply;
            }
            for (int i = 0; i < 6; ++i) {
                _currJointVel[i] = list[1 + i].toFloat();
            }
            break;
        }
        case 2214: {
            auto list = msg.split(",");
            if (list.size() < 7) {
                qWarning(logMecaAdapter()) << "size error with 2214" << reply;
            }
            for (int i = 0; i < 6; ++i) {
                _currTwist[i] = list[1 + i].toFloat();
            }
            break;
        }
    }
}

// --------------------------------------------------------------------------
// SLOTS FUNCTIONS
void mecademic::MecaAdapter::_onControlErrorOccurred(
    QAbstractSocket::SocketError socketError) {
    qCritical(logMecaAdapter()) << "[Control   ] errorOccurred:" << socketError;
    emit abort();
}


void mecademic::MecaAdapter::_onMonitoringErrorOccurred(
    QAbstractSocket::SocketError socketError) {
    qCritical(logMecaAdapter()) << "[Monitoring] errorOccurred:" << socketError;
    emit abort();
}


void mecademic::MecaAdapter::_onControlReadyRead() {
    // all message arrived before this function is abort will be executed
    // immediately without waiting for a new call to this function
    while (!_controlSocket->atEnd()) {
        auto data    = _controlOverflow + _controlSocket->readAll();
        auto replies = data.split('\x00');
        if (data[data.size() - 1] != '\x00') {
            _controlOverflow.append(replies.takeLast());
        } else {
            _controlOverflow.clear();
        }
        for (auto& reply : replies) {
            if (reply.size() > 0) {
                _processControlReply(reply);
            }
        }
    }
}


void mecademic::MecaAdapter::_onMonitoringReadyRead() {
    // all message arrived before this function is abort will be executed
    // immediately without waiting for a new call to this function
    while (!_monitoringSocket->atEnd()) {
        auto data    = _monitoringOverflow + _monitoringSocket->readAll();
        auto replies = data.split('\x00');
        if (data[data.size() - 1] != '\x00') {
            qDebug() << "reply incomplete";
            _monitoringOverflow.append(replies.takeLast());
        } else {
            _monitoringOverflow.clear();
        }
        for (auto& reply : replies) {
            if (reply.size() > 0) {
                _processMonitoringReply(reply);
            }
        }
        emit feedback(_currPose, _currTwist);
    }
}
