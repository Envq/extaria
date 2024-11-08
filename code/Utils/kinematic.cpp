#include "kinematic.h"

#include <QDebug>
#include <QThread>
#include <QtMath>

Q_LOGGING_CATEGORY(logKinematic, "Kinematic")


// ==========================================================================
// MOBILE XYZ rotation convention
QVector<float> teleop::matrix_to_poseXYZ(const QMatrix4x4& matrix) {
    // Get position
    const float x = matrix(0, 3);
    const float y = matrix(1, 3);
    const float z = matrix(2, 3);

    // Get orientation
    float r, p, w;
    if (matrix(0, 2) >= 1.0) {
        p = M_PI * 0.5;
        r = qAtan2(matrix(1, 0), matrix(1, 1));
        w = 0;
    } else if (matrix(0, 2) <= -1.0) {
        p = -M_PI * 0.5;
        r = -qAtan2(matrix(1, 0), matrix(1, 1));
        w = 0;
    } else {
        p = qAsin(matrix(0, 2));
        r = qAtan2(-matrix(1, 2), matrix(2, 2));
        w = qAtan2(-matrix(0, 1), matrix(0, 0));
    }

    // Conversion to degrees
    r = r * 180.0 / M_PI;
    p = p * 180.0 / M_PI;
    w = w * 180.0 / M_PI;

    return {x, y, z, r, p, w};
}


QMatrix4x4 teleop::poseXYZ_to_matrix(const QVector<float>& pose) {
    const float a  = pose[3] * M_PI / 180.0;  // [rad]
    const float b  = pose[4] * M_PI / 180.0;  // [rad]
    const float c  = pose[5] * M_PI / 180.0;  // [rad]
    const float ca = qCos(a);
    const float sa = qSin(a);
    const float cb = qCos(b);
    const float sb = qSin(b);
    const float cc = qCos(c);
    const float sc = qSin(c);

    // clang-format off
    return { cb*cc,                   -cb*sc,      sb,  pose[0],
             ca*sc+sa*sb*cc,  ca*cc-sa*sb*sc,  -sa*cb,  pose[1],
             sa*sc-ca*sb*cc,  sa*cc+ca*sb*sc,   ca*cb,  pose[2],
                          0,               0,       0,       1};
    // clang-format on
}


// ==========================================================================
// MOBILE ZYX rotation convention
QVector<float> teleop::matrix_to_poseZYX(const QMatrix4x4& matrix) {
    // Get position
    const float x = matrix(0, 3);
    const float y = matrix(1, 3);
    const float z = matrix(2, 3);

    // Get orientation
    float r, p, w;
    if (matrix(2, 0) >= 1.0) {
        p = -M_PI * 0.5;
        w = qAtan2(-matrix(1, 2), matrix(1, 1));
        r = 0;
    } else if (matrix(2, 0) <= -1.0) {
        p = M_PI * 0.5;
        w = -qAtan2(matrix(1, 2), matrix(1, 1));
        r = 0;
    } else {
        p = qAsin(-matrix(2, 0));
        w = qAtan2(matrix(1, 0), matrix(0, 0));
        r = qAtan2(matrix(2, 1), matrix(2, 2));
    }

    // Conversion to degrees
    r = r * 180.0 / M_PI;
    p = p * 180.0 / M_PI;
    w = w * 180.0 / M_PI;

    return {x, y, z, w, p, r};
}


QMatrix4x4 teleop::poseZYX_to_matrix(const QVector<float>& pose) {
    const float a  = pose[5] * M_PI / 180.0;  // [rad]
    const float b  = pose[4] * M_PI / 180.0;  // [rad]
    const float c  = pose[3] * M_PI / 180.0;  // [rad]
    const float ca = qCos(a);
    const float sa = qSin(a);
    const float cb = qCos(b);
    const float sb = qSin(b);
    const float cc = qCos(c);
    const float sc = qSin(c);

    // clang-format off
    return {cb*cc,  cc*sa*sb-ca*sc,  sa*sc+ca*cc*sb,  pose[0],
            cb*sc,  ca*cc+sa*sb*sc,  ca*sb*sc-cc*sa,  pose[1],
              -sb,           cb*sa,           ca*cb,  pose[2],
                0,               0,               0,       1};
    // clang-format on
}


QVector<float> teleop::matrix_to_poseXYZ_fixed(const QMatrix4x4& matrix) {
    auto output = matrix_to_poseZYX(matrix);
    return {output[0], output[1], output[2], output[5], output[4], output[3]};
}


QMatrix4x4 teleop::poseXYZ_to_matrix_fixed(const QVector<float>& pose) {
    QVector<float> input = {pose[0], pose[1], pose[2],
                            pose[5], pose[4], pose[3]};
    return poseZYX_to_matrix(input);
}
