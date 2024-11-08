#include "filters.h"

#include <algorithm>


// --------------------------------------------------------------------------
// SIMPLE MOVING AVERAGE
teleop::SimpleMovingAverage::SimpleMovingAverage(int window, QObject* parent)
    : QObject(parent), _window(window) {
}


QVector<float>
teleop::SimpleMovingAverage::applyFilter(const QVector<float>& data) {
    if (_memory.size() < _window) {
        // cumulative moving average
        _memory.enqueue(data);
        int n = _memory.size();
        for (int i = 0; i < 6; ++i) {
            _result[i] = (data[i] + n * _result[i]) / (n + 1);
        }
    } else {
        // simple moving average
        auto tail = _memory.dequeue();
        _memory.enqueue(data);
        for (int i = 0; i < 6; ++i) {
            _result[i] += (data[i] - tail[i]) / _window;
        }
    }
    return _result;
}

// --------------------------------------------------------------------------
// WEIGHTED MOVING AVERAGE
teleop::WeightedMovingAverage::WeightedMovingAverage(int      window,
                                                     QObject* parent)
    : QObject(parent), _window(window) {
}


QVector<float>
teleop::WeightedMovingAverage::applyFilter(const QVector<float>& data) {
    if (_memory.size() < _window) {
        // cumulative moving average
        _memory.enqueue(data);
        int weight = _memory.size();
        _denominator += weight;
        for (int i = 0; i < 6; ++i) {
            _numerator[i] += weight * data[i];
            _total[i] += data[i];
        }

    } else {
        // weighted moving average
        auto tail = _memory.dequeue();
        _memory.enqueue(data);
        for (int i = 0; i < 6; ++i) {
            // Note: this _total is total_n, while the next line _total is
            // total_n+1. So DON'T change order
            _numerator[i] += _window * data[i] - _total[i];
            _total[i] += data[i] - tail[i];
        }
    }
    // clang-format off
    return {_numerator[0] / _denominator,
            _numerator[1] / _denominator,
            _numerator[2] / _denominator,
            _numerator[3] / _denominator,
            _numerator[4] / _denominator,
            _numerator[5] / _denominator};
    // clang-format on
}


// --------------------------------------------------------------------------
// SIMPLE MOVING MEDIAN
teleop::SimpleMovingMedian::SimpleMovingMedian(int window, QObject* parent)
    : QObject(parent), _window(window) {
    _bufferX.resize(_window);
    _bufferY.resize(_window);
    _bufferZ.resize(_window);
    _bufferR.resize(_window);
    _bufferP.resize(_window);
    _bufferW.resize(_window);
}


QVector<float>
teleop::SimpleMovingMedian::applyFilter(const QVector<float>& data) {
    // Insertion phase
    if (_memory.size() < _window) {
        _memory.enqueue(data);
    } else {
        _memory.dequeue();
        _memory.enqueue(data);
    }
    for (int i = 0; i < _memory.size(); ++i) {
        _bufferX[i] = _memory[i][0];
        _bufferY[i] = _memory[i][1];
        _bufferZ[i] = _memory[i][2];
        _bufferR[i] = _memory[i][3];
        _bufferP[i] = _memory[i][4];
        _bufferW[i] = _memory[i][5];
    }
    // Sorting phase
    std::sort(_bufferX.begin(), _bufferX.end());
    std::sort(_bufferY.begin(), _bufferY.end());
    std::sort(_bufferZ.begin(), _bufferZ.end());
    std::sort(_bufferR.begin(), _bufferR.end());
    std::sort(_bufferP.begin(), _bufferP.end());
    std::sort(_bufferW.begin(), _bufferW.end());
    // clang-format off
        return {
            _bufferX[_window / 2],
            _bufferY[_window / 2],
            _bufferZ[_window / 2],
            _bufferR[_window / 2],
            _bufferP[_window / 2],
            _bufferW[_window / 2],
        };
    // clang-format on
}


// --------------------------------------------------------------------------
// BUTTERWORTH LOW PASS
teleop::ButterworthLowPass::ButterworthLowPass(float b0, float b1, float b2,
                                               float a0, float a1,
                                               QObject* parent)
    : QObject(parent), _b0(b0), _b1(b1), _b2(b2), _a0(a0), _a1(a1) {
}


teleop::ButterworthLowPass::ButterworthLowPass(const QVector<float>& parameters,
                                               QObject*              parent)
    : ButterworthLowPass(parameters[0], parameters[1], parameters[2],
                         parameters[3], parameters[4], parent) {
}


QVector<float>
teleop::ButterworthLowPass::applyFilter(const QVector<float>& data) {
    // Get current
    QVector<float> y0 = {0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 6; i++) {
        y0[i] = _a0 * _y1[i] + _a1 * _y2[i] + _b0 * data[i] + _b1 * _x1[i] +
                _b2 * _x2[i];
    }
    // Update memory
    _y2 = _y1;
    _y1 = y0;
    _x2 = _x1;
    _x1 = data;
    return y0;
}
