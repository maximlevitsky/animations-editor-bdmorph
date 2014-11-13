#ifndef VECTOR2D_H
#define VECTOR2D_H

#include <cmath>
#include <complex>
#include <QPoint>
using namespace std;

// adapted from http://www.terathon.com/code/vector2d.html

template<class T>
class Vector2D {
    public:
        T   x;
        T   y;

        void print(char *tag = "") const {
            qWarning("%s %g %g", tag, x, y);
        }

        QPointF toQPoint() const {
            return QPointF(x,y);
        }

        T cross(const Vector2D<T> &v) const {
            return x*v.y - v.x*y;
        }

        T norm() const { return sqrtf(normSquared()); }
        T normSquared() const { return x*x+y*y; }

        Vector2D<T>() : x(0), y(0) {}

        Vector2D<T>(complex<T> v) : x(real(v)), y(imag(v)) {}

        Vector2D<T>(QPointF p) : x(p.x()), y(p.y()) {}

        Vector2D<T>(T r, T s) {
            x = r;
            y = s;
        }

        Vector2D<T>& set(T r, T s) {
            x = r;
            y = s;
            return (*this);
        }

        T& operator [](long k) {
            return ((&x)[k]);
        }

        const T& operator [](long k) const {
            return ((&x)[k]);
        }

        Vector2D<T>& operator +=(const Vector2D<T>& v) {
            x += v.x;
            y += v.y;
            return (*this);
        }

        Vector2D<T>& operator -=(const Vector2D<T>& v) {
            x -= v.x;
            y -= v.y;
            return (*this);
        }

        Vector2D<T>& operator *=(T t) {
            x *= t;
            y *= t;
            return (*this);
        }

        Vector2D<T>& operator /=(T t) {
            T f = 1.0F / t;
            x *= f;
            y *= f;
            return (*this);
        }

        Vector2D<T>& operator &=(const Vector2D<T>& v) {
            x *= v.x;
            y *= v.y;
            return (*this);
        }

        Vector2D<T> operator -() const {
            return (Vector2D<T>(-x, -y));
        }

        Vector2D<T> operator +(const Vector2D<T>& v) const {
            return (Vector2D<T>(x + v.x, y + v.y));
        }

        Vector2D<T> operator -(const Vector2D<T>& v) const {
            return (Vector2D<T>(x - v.x, y - v.y));
        }

        Vector2D<T> operator *(T t) const {
            return (Vector2D<T>(x * t, y * t));
        }

        Vector2D<T> operator /(T t) const {
            T f = 1.0F / t;
            return (Vector2D<T>(x * f, y * f));
        }

        T operator *(const Vector2D<T>& v) const {
            return (x * v.x + y * v.y);
        }

        T dot(const Vector2D<T>& v) const {
            return (*this)*v;
        }

        Vector2D<T> operator &(const Vector2D<T>& v) const {
            return (Vector2D<T>(x * v.x, y * v.y));
        }

        bool operator ==(const Vector2D<T>& v) const {
            return ((x == v.x) && (y == v.y));
        }

        bool operator !=(const Vector2D<T>& v) const {
            return ((x != v.x) || (y != v.y));
        }

        Vector2D<T>& normalize(void) {
            return (*this /= sqrtf(x * x + y * y));
        }

        Vector2D<T>& rotate(T angle);

        complex<T> toComplex() { return complex<T>(x,y); }
};

template<class T>
T rotationAngle(Vector2D<T> v1, Vector2D<T> v2);

template<class T>
class Point2D : public Vector2D<T> {
    public:

        Point2D<T>() : Vector2D<T>(0,0) {}

        Point2D<T>(T r, T s) : Vector2D<T>(r, s) {}

        Point2D<T>(complex<T> v) : Vector2D<T>(v) {}

        Point2D<T>& operator =(const Vector2D<T>& v) {
            this->x = v.x;
            this->y = v.y;
            return (*this);
        }

        Point2D<T>& operator *=(T t) {
            this->x *= t;
            this->y *= t;
            return (*this);
        }

        Point2D<T>& operator /=(T t) {
            T f = T(1.0) / t;
            this-> x *= f;
            this->y *= f;
            return (*this);
        }

        Point2D<T> operator -() const {
            return (Point2D<T>(-this->x, -this->y));
        }

        Point2D<T> operator +(const Vector2D<T>& v) const {
            return (Point2D<T>(this->x + v.x, this->y + v.y));
        }

        Point2D<T> operator -(const Vector2D<T>& v) const {
            return (Point2D<T>(this->x - v.x, this->y - v.y));
        }

        Vector2D<T> operator -(const Point2D<T>& p) const {
            return (Vector2D<T>(this->x - p.x, this->y - p.y));
        }

        Point2D<T> operator *(T t) const {
            return (Point2D<T>(this->x * t, this->y * t));
        }

        Point2D<T> operator /(T t) const {
            T f = T(1.0) / t;
            return (Point2D<T>(this->x * f, this->y * f));
        }

        T distance(Point2D<T> p2) const {
            return (*this-p2).norm();
        }

        T distanceSquared(Point2D<T> p2) const {
            return (*this-p2).normSquared();
        }
};

template<class T>
inline Vector2D<T> operator *(T t, const Vector2D<T>& v) {
    return (Vector2D<T>(t * v.x, t * v.y));
}

template<class T>
inline Point2D<T> operator *(T t, const Point2D<T>& p) {
    return (Point2D<T>(t * p.x, t * p.y));
}

typedef Vector2D<double> Vector2;
typedef Point2D<double> Point2;

#endif // Vector2D<T>_H
