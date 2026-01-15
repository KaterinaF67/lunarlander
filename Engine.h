#ifndef ENGINE_H
#define ENGINE_H

#include "Vector.h"
#include "LanderState.h"

class Engine {
public:
    Vector rel_pos; // локальная
    float dir_angle; // лок(рад)
    float throttle_cmd;     // 0..100 % — команда
    float throttle_actual;  // 0..100 % — фактическая

    float max_thrust;

    Engine(): rel_pos(0,0), dir_angle(0), throttle_cmd(0), throttle_actual(0), max_thrust(0){}

    Engine(Vector pos, float dir, float maxT): rel_pos(pos), dir_angle(dir), throttle_cmd(0), throttle_actual(0), max_thrust(maxT){}

    // Сила в мировых
    Vector getForce(const LanderState& state) const {
        float global_ang = state.angle + dir_angle;
        Vector dir = Vector::fromAngle(global_ang);
        float F = max_thrust * (throttle_actual / 100.0f);
        return dir * F;
    }

    // Момент 
    float getTorque(const LanderState& state) const {
        Vector r_local = rel_pos - state.com;
        Vector r_world = Vector::rot(r_local, state.angle);
        Vector F = getForce(state);
        return r_world.cross(F);
    }
};

#endif