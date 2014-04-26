#include <cmath>
#include "logspiral.h"

template struct LogSpiral<double>;
template struct LogSpiral<float>;

/*template
LogSpiral<double> fitSpiral(Point2D<double> initialPoint, Point2D<double> finalPoint,
                    Point2D<double> initialDerivative);
template
LogSpiral<float> fitSpiral(Point2D<float> initialPoint, Point2D<float> finalPoint,
                    Point2D<float> initialDerivative);*/

template<class T>
LogSpiral<T> fitSpiral(Point2D<T> s0, Point2D<T> s1, Vector2D<T> t0, Vector2D<T> t1) {
    // Implementation of algorithms from sections 2-3 of the paper
    // "Approximation of Logarithmic Spirals" by Baumgarten and Farin (1997)
    // t0 points into the curve, t1 points out

    Vector2D<T> s01 = s1 - s0;
    LogSpiral<T> result;

    // Test for parallel
    Vector2D<T> n0 = t0, n1 = t1;
    n0.normalize();
    n1.normalize();

    // Equation (8) from paper -- the trick is that they figure out the position of the
    // log spiral center point from the angle that tangents make with the line between control points
    T gamma0 = rotationAngle(t0, s01); // make sure these angles are the right orientation
    T gamma1 = rotationAngle(s01, t1);

    // We need to solve equation (11) to find the exponent k controlling log spiral expansion;
    // from there, it is surprisingly easy to find the center point
    T b1 = sin(gamma0);
    T b2 = cos(gamma0);
    T b3 = -sin(gamma1);
    T b4 = cos(gamma1);

    // My handwritten notes used these letters instead...
    T &a = b1, &b = b2, &c = b3, &d = b4;

    //qWarning("b1 = %g\nb2 = %g\nb3 = %g\nb4 = %g",b1,b2,b3,b4);

    // From a cute geometric argument in the paper (p518)
    T phi = gamma0 + gamma1;
    //qWarning("phi = %g", phi);

    if (phi <= 0 || phi >= 2*M_PI) {
        LogSpiral<T> backward = fitSpiral(s1,s0,-t1,-t0);
        result.p0 = backward.p0;
        result.c = -backward.c;
        result.alpha = -backward.alpha;
        return result;
    }

    if ((n0+n1).norm() < 1e-8) { // directions are parallel
        T ratio = exp(-M_PI/tan(gamma1));
        T frac = ratio / (1+ratio);
        result.p0 = frac*s0 + (1-frac)*s1;
        result.alpha = M_PI;
        result.c = -log(ratio);

        return result;
    }

    // We compute "rational linear function" asymptote values (p519)
    T x = -b4/b3, y = b1/b3, x0 = -b2/b1;

    //qWarning("x = %g\ny = %g\nx0 = %g",x,y,x0);

    // The paper's conditions for which root to choose are incorrect, sadly
    // We compute both roots and figure it out later...

    T k = 0, k1, k2;
    bool increasing = a*d-b*c > 0;

    if (y < 0) { // unique solution
        //qWarning("Unique solution");
        T rhs;
        if (increasing) rhs = x - 1; // figure out which side of x to start on
        else rhs = x + 1;

        while (evaluateFunction(rhs,phi,b1,b2,b3,b4) > 0) // we want the rational part to be positive
            rhs = (rhs + x) / 2;

        T rhsEval = evaluateFunction(rhs,phi,b1,b2,b3,b4);

        T lhs = increasing? rhs - 1 : rhs + 1;
        while (rhsEval*evaluateFunction(lhs,phi,b1,b2,b3,b4) > 0 && abs(rhsEval) > 1e-9)
            lhs = lhs + (lhs - rhs);

        k = k1 = k2 = findRoot(phi, b1, b2, b3, b4, lhs, rhs);

        complex<T> z1(b1,b2), z2(b3,b4), div = z1/z2;

        if (fabs(imag(div)) < 1e-8) { // ratio is a constant!
            T r = real(div);

            k = k1 = k2 = log(r)/phi;
        }
    } else if (!increasing) { // solutions are on opposite sides of x
        //qWarning("Opposite sides: %g", x);
        T rhs = x - 1;

        int count = 0;
        bool killed = false;
        while ((b1*rhs+b2)/(b3*rhs+b4) > 0 && !killed) {
            if (++count > 100) killed = true;
            rhs = (rhs + x) / 2;
        }

        T lhs = rhs - 1;
        while (evaluateFunction(lhs, phi, b1, b2, b3, b4) > 0)
            lhs = lhs + (lhs - rhs);

        k1 = killed? x : findRoot(phi, b1, b2, b3, b4, lhs, rhs);

        lhs = x + 1;
        count = 0;
        killed = false;
        while (evaluateFunction(lhs, phi, b1, b2, b3, b4) > 0 && !killed) {
            if (++count > 100) killed = true;
            lhs = (lhs + x) / 2;
        }

        rhs = lhs + 1;
        while (evaluateFunction(rhs, phi, b1, b2, b3, b4) < 0)
            rhs = rhs + (rhs - lhs);

        k2 = killed? x : findRoot(phi, b1, b2, b3, b4, lhs, rhs);
    } else { // solutions are on the same side of x ... ugh -- increasing
        //qWarning("Same side");
        T expValue = exp(x*phi);
        bool onLeft = expValue > y;

        T middleK = onLeft? x-1 : x+1;

        // Newton iterations to find a point with opposite sign
        T multiplier = onLeft? 1 : -1;
        while (evaluateFunction(middleK,phi,b1,b2,b3,b4)*multiplier < 0)
            middleK -= evaluateDerivative(middleK,phi,b1,b2,b3,b4) / evaluateSecondDerivative(middleK,phi,b1,b2,b3,b4);

        // Now, we need brackets for the two roots and solve
        T midEval = evaluateFunction(middleK,phi,b1,b2,b3,b4);
        T l = (midEval + x)/2;
        while (midEval * evaluateFunction(l,phi,b1,b2,b3,b4) > 0)
            l = (l+x)/2;
        k1 = findRoot(phi, b1, b2, b3, b4, l, middleK);

        l = middleK + (middleK - x);
        while (midEval * evaluateFunction(l,phi,b1,b2,b3,b4) > 0)
            l = l + (l - x);
        k2 = findRoot(phi, b1, b2, b3, b4, l, middleK);
    }

    //qWarning("k1 = %g\nk2 = %g", k1, k2);

    LogSpiral<T> result1 = computeSpiralParameters(s0,s1,t0,t1,k1,gamma0,phi);
    LogSpiral<T> result2 = computeSpiralParameters(s0,s1,t0,t1,k2,gamma0,phi);

    complex<T> z(result1.c,result1.alpha);
    complex<T> center = result1.p0.toComplex();
    complex<T> initialPoint = s0.toComplex();
    complex<T> finalPoint = s1.toComplex();
    complex<T> v0 = z*(initialPoint - center);
    complex<T> v1 = z*(finalPoint - center);
    Vector2D<T> t0Parallel(v0);
    Vector2D<T> t1Parallel(v1);

    T dot0 = t0.dot(t0Parallel);
    T dot1 = t1.dot(t1Parallel);

    if (dot0 > 0 && dot1 > 0) result = result1;
    else result = result2;

    if (k1 == numeric_limits<T>::max()) result = result2;
    if (k2 == numeric_limits<T>::max()) result = result1;

    //qWarning("z = %g + i*%g",result.c, result.alpha);

    return result;
}



template
LogSpiral<double> fitSpiral(Point2D<double> s0, Point2D<double> s1, Vector2D<double> t0, Vector2D<double> t1);
template
LogSpiral<float> fitSpiral(Point2D<float> s0, Point2D<float> s1, Vector2D<float> t0, Vector2D<float> t1);
