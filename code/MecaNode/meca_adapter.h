#ifndef MECA_ADAPTER_H
#define MECA_ADAPTER_H

#include "logs.h"

#include <QLoggingCategory>
#include <QSet>
#include <QTcpSocket>

Q_DECLARE_LOGGING_CATEGORY(logMecaAdapter)


// ==========================================================================
// INFO: Based on Firmware 8.4
namespace mecademic {

// ==========================================================================
// Holds the current meca status (0 or 1)
// Note that inPauseMotion = 1 if and only if a PauseMotion or a ClearMotion was
// sent, or if the robot is in error mode.
struct MecaStatus {
    bool activated     = false;
    bool homed         = false;
    bool inSimulation  = false;
    bool inError       = false;
    bool inPauseMotion = false;
    bool endOfBlock    = false;
    bool endOfMovement = false;
};

QDebug operator<<(QDebug debug, const MecaStatus& status);

// ==========================================================================
class MecaAdapter : public QObject {
    Q_OBJECT

  public:
    explicit MecaAdapter(QObject* parent = 0);
    ~MecaAdapter();

    // ----------------------------------------------------------------------
    // MOTION COMMANDS
    // They do not generate a direct response and the only way to know when
    // exactly a certain motion command has been executed is to use the command
    // SetCheckpoint

    // Add a time delay after a motion command
    //   Raw command: Delay(t)
    void delay(float sec);

    // Defines a checkpoint in the motion queue
    //        Limits: min=1, max=8000
    //   Raw command: SetCheckpoint(n)
    void     setCheckpoint(int n);
    unsigned addCheckpoint();  // add new checkpoint with n incrementaly

    // Defines the pose the TCP(TRF) w.r.t. the flange(FRF) in mm
    //   Raw command: SetTRF(x,y,z,r,p,w)
    void setTCP(const QVector<float>& tcp);

    // Limits the Cartesian Linear velocity of the TCP
    // Influence: MoveLin, MoveLinRelTRF, MoveLinRelWRF
    //   Linear limits: min=0.001, max=1000, default=150  [mm/s]
    //     Raw command: SetCartLinVel(v)
    void setCartLinVel(float v = 150);

    // Limits the Cartesian Angular velocity of the TCP
    // Influence: MoveLin, MoveLinRelTRF, MoveLinRelWRF
    //  Angular limits: min=0.001, max= 300, default= 45  [degrees/s]
    //     Raw command: SetCartAngVel(w)
    void setCartAngVel(float w = 45);

    // Limits the Cartesian acceleration (both linear and angular) of the TCP
    // Influence: MoveLin, MoveLinRelTRF, MoveLinRelWRF, MoveLinVelTRF,
    // MoveLInVelWRF
    //          Limits: min=0.001, max=600, default=50  [%]
    //     Raw command: SetCartAcc(p)
    void setCartAcc(float perc = 50);

    // Limits the angular velocity of the robot joints
    // Influence: MoveJoints, MovePose
    //          Limits: min=0.001, max=100, default=25  [%]
    //     Raw command: SetJointVel(p)
    void setJointVel(float perc = 25);

    // Limits the acceleration of the robot joints
    // Influence: MoveJoints, MovePose, MoveJointsVel
    //          Limits: min=0.001, max=150, default=100  [%]
    //     Raw command: SetJointAcc(p)
    void setJointAcc(float perc = 100);

    // Set the percentage of blending (or disable it with 0%)
    //          Limits: min=0, max=100, default=100  [%]
    //     Raw command: SetBlending(p)
    void setBlending(float perc = 100);

    // Sets the desired robot posture config to be observed in the MovePose
    //          Values: 1 or -1
    //     Raw command: SetConf(cs,ce,cw)
    void setConf(int shoulder, int elbow, int wrist);

    // Sets the desired turn configuration for joint 6, which can rotate +-100
    // revolutions. This command is useful only if you have a wired end-effector
    // with sufficiently long cables to allow joint 6 to rotate more than +-180
    // degrees. It is disabled by default.
    // The desired range for joint 6 will be: -180+turn*360 < j6 <= 180+turn*360
    //          Limits: min=-100, max=100
    //     Raw command: SetConfTurn(ct)
    void setConfTurn(int turn);

    // Enables or disables the automatic robot posture configuration selection
    // Default enabled. SetConf automatically disable it
    //     Raw command: SetAutoConf(e)
    void setAutoConf(bool enable);

    // Enables or disables the automatic turn selection for joint 6
    // Default enabled. SetConfTurn automatically disable it
    //     Raw command: SetAutoConfTurn(e)
    void setAutoConfTurn(bool enable);

    // Simultaneously rotates the robot joints to the specified joint set
    //       j1 limits: min=  -175, max=  175  [degrees]
    //       j2 limits: min=   -70, max=   90  [degrees]
    //       j3 limits: min=  -135, max=   70  [degrees]
    //       j4 limits: min=  -170, max=  170  [degrees]
    //       j5 limits: min=  -115, max=  115  [degrees]
    //       j6 limits: min=-36000, max=36000  [degrees]
    //     Raw command: MoveJoints(j1,j2,j3,j4,j5,j6)
    void moveJoints(const QVector<float>& joints);

    // Linear Move of the TCP (TRF w.r.f WRF) [mm, degrees]
    //   Raw command: MoveLin(x,y,z,r,p,w)
    void moveLin(const QVector<float>& pose);

    // Linear Move from the TCP (desired TRF w.r.f current TRF) [mm, degrees]
    //   Raw command: MoveLinRelTRF(x,y,z,r,p,w)
    void moveLinRel(const QVector<float>& pose);

    // Joint Move of the TCP (TRF w.r.f WRF) [mm, degrees]
    // After the robot is homed, Monitoring Port transmits periodically data, at
    // the rate specified by the SetMonitoringInterval command
    //   Raw command: MovePose(x,y,z,r,p,w)
    void movePose(const QVector<float>& pose);

    // Sets the timeout after a velocity-mode motion command, after which all
    // joint speeds will be set to zero unless another velocity-mode motion
    // command is received.
    //        Limits: min=0.001, max=1, default=0.050  [sec]
    //   Raw command: SetVelTimeout(t)
    void setVelTimeout(float sec);

    // It use degrees/s. The admisible ranges for the joint velocities are:
    //       j1 limits: min=-150, max=150  [degrees/s]
    //       j2 limits: min=-150, max=150  [degrees/s]
    //       j3 limits: min=-180, max=180  [degrees/s]
    //       j4 limits: min=-300, max=300  [degrees/s]
    //       j5 limits: min=-300, max=300  [degrees/s]
    //       j6 limits: min=-300, max=300  [degrees/s]
    //     Raw command: MoveJointsVel(j1,j2,j3,j4,j5,j6)
    void moveJointVel(const QVector<float>& jointsVel);

    // Move robot with the specified Cartesian velocity
    //     Linear: min=-1000, max=1000  [mm/s]
    //    Angular: min=-300,  max=300   [degree/s]
    //   Raw command: MoveLinVelWRF(x,y,z,r,p,w)
    void moveTwist(const QVector<float>& twist);

    // Limits the velocity of the gripper fingers. max it's about 100 mm/s
    //          Linear: min=5, max=100, default=50  [%]
    //     Raw command: SetGripperVel(p)
    void setGripperVel(float perc = 50);

    // Limits the velocity of the gripper fingers. max it's about 40 N
    //          Linear: min=5, max=100, default=50  [%]
    //     Raw command: SetGripperForce(p)
    void setGripperForce(float perc = 50);

    // Open the gripper fingers.
    //     Raw command: GripperOpen
    void gripperOpen();

    // Close the gripper fingers.
    //     Raw command: GripperClose
    void gripperClose();

    // ----------------------------------------------------------------------
    // REQUEST COMMANDS
    // They are executed immediately and return a specific response

    // Activate all motors and disables the brakes on joints 1, 2, and 3
    // It must be sent before homing is started
    //     Raw command: ActivateRobot
    //        Response: [2000][Motors activated.]
    //                  [2001][Motors already activated.]
    void activateRobot();

    // Disables all motors and engages the brakes on joints 1, 2, and 3
    // WARNING: It must not be sent while the robot is moving
    // When this is executed, the robot loses its homing
    //     Raw command: DeactivateRobot
    //        Response: [2004][Motors deactivated.]
    void deactivateRobot();

    // The Meca500 supports a simulation mode in which all of the robot’s
    // hardware functions normally, but none of the motors move.
    //     Raw command: ActivateSim
    //        Response: [2045][The simulation mode is enabled.]
    void activateSim();

    // The Meca500 supports a simulation mode in which all of the robot’s
    // hardware functions normally, but none of the motors move.
    //     Raw command: DeactivateSim
    //        Response: [2046][The simulation mode is disabled.]
    void deactivateSim();

    // Starts the robot and gripper homing process (takes about 3 sec)
    //     Raw command: Home
    //        Response: [2002][Homing done.]
    //                  [2003][Homing already done.]
    //                  [1032][Homing failed because joints are outside limits.]
    //                  [1014][Homing failed.]
    void homing();

    //    it is necessary to reset the errors to be able to use the robot when
    //    it has finished error mode
    //     Raw command: ResetError
    //        Response: [2005][The error was reset.]
    //                  [2006][There was no error to reset.]
    void resetError();

    // Pauses the robot movement by decelerating. The movement can be resumed
    // with the ResumeMotion command. If a motion error occurs while the robot
    // is at pause (e.g., if another moving body hits the robot), the motion is
    // cleared and can no longer be resumed.
    //     Raw command: PauseMotion
    //        Response: [2042][Motion paused.]
    //                  [3004][End of movement.]
    void pauseMotion();

    // Resumes the robot movement, if it was previously paused with the command
    // PauseMotion. More precisely, the robot end-effector resumes the rest of
    // the trajectory from the pose where it was brought to a stop(after
    // deceleration), unless an error occurred after the PauseMotion or the
    // robot was deactivated and then reactivated.
    // Note: this command must be sent after ClearMotion and ResetError.
    //     Raw command: ResumeMotion
    //        Response: [2043][Motion resumed.]
    void resumeMotion();

    // Stop the robot movement by decelerating like the PauseMotion command.
    // However, if the robot is stopped in the middle of a trajectory, the rest
    // of the trajectory is deleted. As is the case with PauseMotion, you need
    // to send the command ResumeMotion to make the robot ready to execute new
    // motion commands.
    //     Raw command: ClearMotion
    //        Response: [2044][The motion was cleared.]
    //                  [3004][End of movement.]
    void clearMotion();

    // Get the number of motion commands that are currently in the motion queue
    //     Raw command: GetCmdPendingCount
    //        Response: [2080][n]
    void getCmdPendingCount();

    // Set the time interval at which real-time feedback from the robot is sent
    // from the robot over MonitoringPort
    //          Linear: min=0.001, max=1, default=0.015  [sec]
    //     Raw command: SetMonitoringInterval(t)
    void setMonitoringInterval(float sec = 0.015);

    // Calls ResetError, ClearMotion and ResumeMotion
    void reset();

    // ----------------------------------------------------------------------
    // CUSTOM COMMANDS

    // Try to Connect to the Control and Monitoring port, abort otherwise.
    // This method calls SetRTC and SetRealTimeMonitoring(2210,2211,2212,2214)
    void initCommunication(const QString& address);

    // return true if the robot is activated
    bool isActivated();

    // return true if the robot is homed
    bool isHomed();

    // return true if robotStopped AND motionQueueEmpty, otherwise false
    bool isEndOfBlockReached();

    // get the list of checkpoints reached
    QSet<unsigned> getCheckpointReached();

    MecaStatus     getRobotStatus() const;
    QVector<float> getJoints() const;
    QVector<float> getPose() const;
    QVector<float> getJointVel() const;
    QVector<float> getTwist() const;
    unsigned       getTime() const;

  private:
    // Connection
    const unsigned _controlPort      = 10000;
    const unsigned _monitoringPort   = 10001;
    QTcpSocket*    _controlSocket    = nullptr;
    QTcpSocket*    _monitoringSocket = nullptr;

    // Goals Reached State from Control
    unsigned       _checkpointRequest = 1;
    QSet<unsigned> _checkpointsReached{};       // [2230]
    bool           _endOfBlockReached = false;  // [3012]
    bool           _activated         = false;  // [2000][2001]
    bool           _homed             = false;  // [2002][2003]

    // Current State from Monitoring
    unsigned       _currTimeStamp = 0;                  // [2230]
    MecaStatus     _currRobotStatus{};                  // [2007]
    QVector<float> _currPose     = {0, 0, 0, 0, 0, 0};  // [2211]
    QVector<float> _currTwist    = {0, 0, 0, 0, 0, 0};  // [2214]
    QVector<float> _currJoints   = {0, 0, 0, 0, 0, 0};  // [2210]
    QVector<float> _currJointVel = {0, 0, 0, 0, 0, 0};  // [2212]

    // Socket
    QByteArray _controlOverflow;
    QByteArray _monitoringOverflow;

    // Enable the transmission of other real-time data over the monitoring port:
    // [2210] JointPos, [2211] CartPose, [2212] JointVel, [2214] Cartvel
    //     Raw command: SetRealTimeMonitoring(n1, n2, ...)
    void _setRealTimeMonitoring();

    // Sets the internal clock of the robot to UTC. It is essential if if you
    // want all timestamps in your log files to be with respect to UTC. t is the
    // Epoch time as defined in Unix (i.e., number of seconds since 00:00:00 UTC
    // January 1, 1970)
    //     Raw command: SetRTC(t)
    void _setTime();

    // Add newline character to cmd and sent it to the control socket
    void _sendCommand(const QString& cmd);

    // Used to safely normalize the parameters to be sent to the Robot
    float _norm(float val, float min, float max);

    // Used to safely normalize the config parameters  to be sent to the Robot
    int _normConf(int val);

    // process the reply on Control Port
    void _processControlReply(const QByteArray& reply);

    // process the reply on Monitoring Port
    void _processMonitoringReply(const QByteArray& reply);

  private slots:
    void _onControlErrorOccurred(QAbstractSocket::SocketError socketError);
    void _onMonitoringErrorOccurred(QAbstractSocket::SocketError socketError);
    void _onControlReadyRead();
    void _onMonitoringReadyRead();

  signals:
    void abort();
    void feedback(const QVector<float>& pose, const QVector<float>& twist);
};

}  // namespace mecademic


#endif  // MECA_ADAPTER_H
