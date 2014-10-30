#ifndef LOGSPIRAL_H
#define LOGSPIRAL_H

#include "vector2d.h"
#include <complex>
#include <limits>
#include <QDebug>
using namespace std;

#undef min
#undef max

template <class T>
struct LogSpiral {
    Point2D<T> p0;
    T c;
    T alpha;

    Point2D<T> evaluate(Point2D<T> p, T t) {
        T exponential = exp(c*t);
        Vector2D<T> diff = p-p0;

        T cosine = cos(t*alpha);
        T sine = sin(t*alpha);

        Vector2D<T> rotated(cosine*diff.x-sine*diff.y,sine*diff.x+cosine*diff.y);

        return p0 + exponential * rotated;
    }
};

#endif // LOGSPIRAL_H
