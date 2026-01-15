#ifndef VECTOR_H
#define VECTOR_H
#include <cmath>

class Vector {
public:
    float x;
    float y;

    Vector(): x(0), y(0){}
    Vector(float x, float y): x(x), y(y){}

    Vector operator+(const Vector& v) const {return {x + v.x, y + v.y}; }
    Vector operator-(const Vector& v) const {return {x - v.x, y - v.y}; }
    Vector operator*(float k) const {return {x*k, y*k}; }
    Vector operator/(float k) const {return {x/k, y/k}; }
    Vector& operator+=(const Vector& v) { x+= v.x; y+= v.y; return *this; }
    Vector& operator-=(const Vector& v) { x-= v.x; y-= v.y; return *this; }
    Vector& operator*=(float k) { x*= k; y*= k; return *this; }
    Vector& operator/=(float k) { x/= k; y/= k; return *this; }

    float len() const {return std::sqrt(x*x + y*y); }

    Vector norm() const {
        float L = len();
        if (L < 1e-6f) return {0,0};
        return {x / L, y / L};
    }

    float dot(const Vector& v) const { return x * v.x + y * v.y; }
    float cross(const Vector& v) const { return x * v.y - y * v.x; }

    float dist(const Vector& v) const {
        float dx = x - v.x;
        float dy = y - v.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    static Vector rot(const Vector& v, float rad) {
        float c = std::cos(rad);
        float s = std::sin(rad);
        return { v.x*c - v.y*s, v.x*s + v.y*c };
    }

    static Vector fromAngle(float rad) {
        return { -sin(rad), cos(rad) };
    }

    Vector proj(const Vector& v) const {
        float L2 = v.x*v.x + v.y*v.y;
        if (L2 < 1e-6f) return {0,0};
        float k = dot(v) / L2;
        return { v.x*k, v.y*k };
    }

    bool isZero(float eps = 1e-6f) const {
        return std::fabs(x) < eps && std::fabs(y) < eps;
    }
};

#endif
