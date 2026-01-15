#ifndef LANDERSTATE_H
#define LANDERSTATE_H

#include "Vector.h"
#include "Constants.h"
//+угол против часовой

class LanderState {
public:
    Vector pos; // положение цм в мире
    Vector v;   
    Vector a;   
    float angle;  // угол корпуса (рад)
    float omega;  // угловая скорость (рад/с)

    float target_angle; // желаемый угол корпуса (рад)

    float fuel_main;
    float fuel_left;
    float fuel_right;

    float t;    

    Vector com;  // цм в локальных
    LanderState(): pos(Constants::WORLD_WIDTH*0.5f, Constants::WORLD_HEIGHT*0.85f),
        v(0,-30), a(0,0), angle(0.0f), omega(0.0f), target_angle(0.0f),
        fuel_main(Constants::FUEL_MAIN), fuel_left(Constants::FUEL_SIDE), fuel_right(Constants::FUEL_SIDE),
        t(0.0f), com(0.0f, 0.0f) {}

    float getDryMass() const {
        return Constants::DRY_MASS;
    }

    float getFuelMass() const {
        return fuel_main + fuel_left + fuel_right;
    }

    float getTotalMass() const {
        return getDryMass() + getFuelMass();
    }

    float getSpeed() const {
        return v.len();
    }

    float getVerticalSpeed() const {
        return v.y;
    }

    float getHorizontalSpeed() const {
        return v.x;
    }

    float angleDeg() const {
        return angle * 180.0f / Constants::PI;
    }
};

#endif
