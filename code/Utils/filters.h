#ifndef FILTERS_H
#define FILTERS_H

#include <QObject>
#include <QQueue>
#include <QVector>


namespace teleop {

class SimpleMovingAverage : public QObject {
    Q_OBJECT

  public:
    SimpleMovingAverage(int window, QObject* parent = nullptr);
    QVector<float> applyFilter(const QVector<float>& data);

  private:
    const int              _window;
    QQueue<QVector<float>> _memory;
    QVector<float>         _result = {0, 0, 0, 0, 0, 0};
};


class WeightedMovingAverage : public QObject {
    Q_OBJECT

  public:
    WeightedMovingAverage(int window, QObject* parent = nullptr);
    QVector<float> applyFilter(const QVector<float>& data);

  private:
    const int              _window;
    QQueue<QVector<float>> _memory;
    float                  _denominator = 0;  // weights sum
    QVector<float>         _total       = {0, 0, 0, 0, 0, 0};
    QVector<float>         _numerator   = {0, 0, 0, 0, 0, 0};
};


class SimpleMovingMedian : public QObject {
    Q_OBJECT

  public:
    SimpleMovingMedian(int window, QObject* parent = nullptr);
    QVector<float> applyFilter(const QVector<float>& data);

  private:
    const int              _window;
    QQueue<QVector<float>> _memory;
    QVector<float>         _bufferX;
    QVector<float>         _bufferY;
    QVector<float>         _bufferZ;
    QVector<float>         _bufferR;
    QVector<float>         _bufferP;
    QVector<float>         _bufferW;
};


class ButterworthLowPass : public QObject {
    Q_OBJECT

  public:
    ButterworthLowPass(float b0, float b1, float b2, float a0, float a1,
                       QObject* parent = nullptr);
    ButterworthLowPass(const QVector<float>& parameters,
                       QObject*              parent = nullptr);
    QVector<float> applyFilter(const QVector<float>& data);

  private:
    const float    _b0;
    const float    _b1;
    const float    _b2;
    const float    _a0;
    const float    _a1;
    QVector<float> _x1 = {0, 0, 0, 0, 0, 0};
    QVector<float> _x2 = {0, 0, 0, 0, 0, 0};
    QVector<float> _y1 = {0, 0, 0, 0, 0, 0};
    QVector<float> _y2 = {0, 0, 0, 0, 0, 0};
};


}  // namespace teleop


#endif  // FILTERS_H
