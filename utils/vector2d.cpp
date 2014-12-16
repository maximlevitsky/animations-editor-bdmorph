#include "vector2d.h"
#include <QDebug>
#include <math.h>

template<class T>
Vector2D<T>& Vector2D<T>::rotate(T angle) { // counterclockwise
    T s = sin(angle);
    T c = cos(angle);

    T nx = c * x - s * y;
    T ny = s * x + c * y;

    x = nx;
    y = ny;

    return (*this);
}

template<class T>
T rotationAngle(Vector2D<T> v1, Vector2D<T> v2) {
    // Apparently atan2 doesn't care about unit-length vectors
    T cosine = v1.dot(v2);
    T sine = v1.cross(v2);
    T angle = atan2(sine, cosine); // atan2 does sine and then cosine

    //qWarning("Vec 1:  %g %g", v1.x, v1.y);
    //qWarning("Vec 2:  %g %g", v2.x, v2.y);
    //qWarning("Original angle: %g (%g,%g)", angle/M_PI, cosine, sine);

    return (angle<0)? angle+2*M_PI : angle; // atan2 does [-pi, pi]
}

template class Vector2D<float>;
template class Vector2D<double>;
template class Point2D<float>;
template class Point2D<double>;
template double rotationAngle(Vector2D<double> v1, Vector2D<double> v2);
template float rotationAngle(Vector2D<float> v1, Vector2D<float> v2);
