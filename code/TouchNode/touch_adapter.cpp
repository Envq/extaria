#include "touch_adapter.h"

#include <QDebug>
#include <QElapsedTimer>
#include <QThread>

Q_LOGGING_CATEGORY(logTouchAdapter, "TouchAdapter")


// ==========================================================================
namespace {

// Touch Callback: update current state
HDCallbackCode HDCALLBACK UpdateStateCallback(void* data) {
    systems3d::TouchState* state = static_cast<systems3d::TouchState*>(data);

    // Begins the haptics frame, which is a block of code within which the
    // device state is guaranteed to be consistent.
    hdBeginFrame(hdGetCurrentDevice());

    hdGetBooleanv(HD_CURRENT_INKWELL_SWITCH, &state->outsideInkwell);

    // Retrieve the current buttons: In order to get the specific button_N
    // state, use a bitmask to test for the HD_DEVICE_BUTTON_N bit
    int buttons = 0;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &buttons);
    state->buttonDown = (buttons & HD_DEVICE_BUTTON_1) ? HD_TRUE : HD_FALSE;
    state->buttonUp   = (buttons & HD_DEVICE_BUTTON_2) ? HD_TRUE : HD_FALSE;

    hdGetFloatv(HD_CURRENT_TRANSFORM, state->poseMat);

    //    hdGetFloatv(HD_CURRENT_VELOCITY, state->velLin);

    //    hdGetFloatv(HD_CURRENT_ANGULAR_VELOCITY, state->velAng);

    // End of the haptics frame
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_DONE;
}

// Touch Callback: generate force feedback
HDCallbackCode HDCALLBACK ForceFeedbackCallback(void* data) {
    HDfloat* force = static_cast<HDfloat*>(data);

    // Begins the haptics frame, which is a block of code within which the
    // device state is guaranteed to be consistent.
    hdBeginFrame(hdGetCurrentDevice());

    hdSetFloatv(HD_CURRENT_FORCE, force);

    // End of the haptics frame
    hdEndFrame(hdGetCurrentDevice());

    return HD_CALLBACK_DONE;
}

// Call this after any HD functions
void _checkHDErrors() {
    HDErrorInfo info = hdGetError();
    if (HD_DEVICE_ERROR(info)) {
        // Check if communication with the device has been interrupted
        if (info.errorCode == HD_COMM_ERROR ||
            info.errorCode == HD_COMM_CONFIG_ERROR ||
            info.errorCode == HD_TIMER_ERROR ||
            info.errorCode == HD_INVALID_PRIORITY ||
            info.errorCode == HD_SCHEDULER_FULL) {
            qCritical(logTouchAdapter()) << hdGetErrorString(info.errorCode);
            emit abort();
        } else {
            qWarning(logTouchAdapter()) << hdGetErrorString(info.errorCode);
        }
    }
}

}  // namespace


// ==========================================================================
systems3d::TouchAdapter::TouchAdapter(QObject* parent) : QObject(parent) {
    qDebug(logTouchAdapter())
        << QThread::currentThreadId() << " | TouchAdapter::TouchAdapter";
}


void systems3d::TouchAdapter::start() {
    // Initialize the device: must be done before any call to HD functions
    _device = hdInitDevice(HD_DEFAULT_DEVICE);
    _checkHDErrors();

    // Check Calibration
    if (hdCheckCalibration() != HD_CALIBRATION_OK) {
        qCritical(logTouchAdapter()) << "Calibration must be done...";
        emit abort();
    } else {
        qInfo(logTouchAdapter()) << "Calibration OK";
    }

    // Start the servo loop scheduler
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    _checkHDErrors();

    // Warmup
    QThread::msleep(50);
}


systems3d::TouchAdapter::~TouchAdapter() {
    qDebug(logTouchAdapter())
        << QThread::currentThreadId() << " | TouchAdapter::~TouchAdapter";
    // For cleanup, unschedule callbacks and stop the servo loop
    hdStopScheduler();
    hdDisableDevice(_device);
}


void systems3d::TouchAdapter::updateState() {
    hdScheduleSynchronous(UpdateStateCallback, &_state,
                          HD_DEFAULT_SCHEDULER_PRIORITY);
    _checkHDErrors();

    if (!_startupDone) {
        if (_state.outsideInkwell) {
            qCritical(logTouchAdapter()) << "Need to start inside the inkwell";
            emit abort();
        }
        _startupDone = true;
    }
}


bool systems3d::TouchAdapter::getButtonDown() {
    return _state.buttonDown;
}


bool systems3d::TouchAdapter::getButtonUp() {
    return _state.buttonUp;
}


QMatrix4x4 systems3d::TouchAdapter::getPoseMatrix() {
    const HDfloat* m = _state.poseMat;
    // clang-format off
    return {m[0],  m[4],  m[8],  m[12],
            m[1],  m[5],  m[9],  m[13],
            m[2],  m[6],  m[10], m[14],
            m[3],  m[7],  m[11], m[15]};
    // clang-format on
}


QVector3D systems3d::TouchAdapter::getPosition() {
    const HDfloat* m = _state.poseMat;
    return {m[12], m[13], m[14]};
}


// QVector3D systems3d::TouchAdapter::getVelocityLinear() {
//    const HDfloat* vel = _state.velLin;
//    return {vel[2], vel[0], vel[1]};
//}


// QVector3D systems3d::TouchAdapter::getVelocityAngular() {
//    const HDfloat* vel = _state.velAng;
//    return {vel[2], vel[0], vel[1]};
//}


void systems3d::TouchAdapter::setForce(const QVector<float>& force) {
    if (_state.outsideInkwell) {
        HDfloat feedback[3] = {force[0], force[1], force[2]};
        hdScheduleSynchronous(ForceFeedbackCallback, &feedback,
                              HD_DEFAULT_SCHEDULER_PRIORITY);
        _checkHDErrors();
    }
}
