#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <QElapsedTimer>
#include <QLoggingCategory>
#include <QMatrix4x4>
#include <QObject>
#include <QVector>

Q_DECLARE_LOGGING_CATEGORY(logKinematic)


namespace teleop {

// ==========================================================================
// Pose: XYZ [mm], Roll-Pitch-Yaw [degrees]
// Rotation convention: mobile XYZ (== fixed ZYX)
QVector<float> matrix_to_poseXYZ(const QMatrix4x4& matrix);
QMatrix4x4     poseXYZ_to_matrix(const QVector<float>& pose);

// ==========================================================================
// Pose: XYZ [mm], Yaw-Pitch-Roll [degrees]
// Rotation convention: mobile ZYX (== fixed XYZ)
QVector<float> matrix_to_poseZYX(const QMatrix4x4& matrix);
QMatrix4x4     poseZYX_to_matrix(const QVector<float>& pose);
QVector<float> matrix_to_poseXYZ_fixed(const QMatrix4x4& matrix);
QMatrix4x4     poseXYZ_to_matrix_fixed(const QVector<float>& pose);

}  // namespace teleop


#endif  // KINEMATICS_H
