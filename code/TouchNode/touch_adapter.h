#ifndef TOUCH_ADAPTER_H
#define TOUCH_ADAPTER_H

#include <HD/hd.h>
#include <QLoggingCategory>
#include <QMatrix4x4>
#include <QObject>
#include <QVector3D>

Q_DECLARE_LOGGING_CATEGORY(logTouchAdapter)


// ==========================================================================
// INFO: max rate = 1000hz
namespace systems3d {

// ==========================================================================
// Holds data retrieved from HDAPI.
struct TouchState {
    HDboolean outsideInkwell = false;  // Status of inkwell switch
    HDboolean buttonDown     = false;  // Status of pressing button1
    HDboolean buttonUp       = false;  // Status of pressing button2
    HDfloat   force[3]       = {};     // [N] Force feedback
    //    HDfloat   velLin[3]   = {};     // [mm/s] Smoothed Linear velocity
    //    HDfloat   velAng[3]   = {};     // [mm/s] Smoothed Angular velocity
    HDfloat poseMat[16] = {};  // [mm] Homogeneous matrix (Colomun Major)
};

// ==========================================================================
class TouchAdapter : public QObject {
    Q_OBJECT

  public:
    explicit TouchAdapter(QObject* parent = 0);
    TouchAdapter(const TouchAdapter&) = delete;
    TouchAdapter(TouchAdapter&&)      = delete;
    ~TouchAdapter();

    void       start();
    void       updateState();
    bool       getButtonDown();
    bool       getButtonUp();
    QMatrix4x4 getPoseMatrix();
    QVector3D  getPosition();
    //    QVector3D  getVelocityLinear();
    //    QVector3D  getVelocityAngular();
    void setForce(const QVector<float>& force);

  private:
    HHD        _device      = 0;
    TouchState _state       = {};
    bool       _startupDone = false;

  signals:
    void abort();
};

}  // namespace systems3d

#endif  // TOUCH_ADAPTER_H
