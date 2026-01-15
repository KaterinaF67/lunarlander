#ifndef LANDER_H
#define LANDER_H

#include "Vector.h"
#include "Engine.h"
#include "LanderState.h"
#include "Constants.h"
#include <algorithm>
#include <cmath>

class Lander {
private:
    LanderState state;

    float hull_width = Constants::HULL_WIDTH;
    float hull_height = Constants::HULL_HEIGHT;
    float landing_leg_length = Constants::LEG_LENGTH;
    float leg_spacing = Constants::LEG_SPACING; 
    Engine* engines[4];


    Engine engine_main_left;
    Engine engine_main_right;
    Engine engine_side_left;
    Engine engine_side_right;

    float inertia = Constants::HULL_INERTIA;
    void bindEngines() {
        engines[0] = &engine_main_left;
        engines[1] = &engine_main_right;
        engines[2] = &engine_side_left;
        engines[3] = &engine_side_right;
    }

    Vector computeCOM() const{
        float dry = Constants::DRY_MASS;
        float fm = state.fuel_main;
        float fl = state.fuel_left;
        float fr = state.fuel_right;

        float total = dry + fm + fl + fr;
        if (total < 1e-5f) total = 1.0f;

        float tank_z = -hull_height* 0.25f;
        float tank_x = hull_width * 0.25f;

        Vector sum(0,0);
        sum += Vector(0,0)* dry;
        sum += Vector(0, tank_z)* fm;
        sum += Vector(-tank_x, tank_z)* fl;
        sum += Vector(+tank_x, tank_z)* fr;

        return sum / total;
    }

    void updateCOM(){
        state.com = computeCOM();
    }

    float computeInertia() const {
        const float m_hull = Constants::DRY_MASS;

        const float I_hull_cm = (m_hull * (hull_width*hull_width + hull_height*hull_height)) / 12.0f;

        // смещение корпуса относительно цм
        const Vector r_hull = Vector(0,0) - state.com;
        float I = I_hull_cm + m_hull * r_hull.dot(r_hull);

        const float tank_z = -hull_height* 0.25f;
        const float tank_x = hull_width* 0.25f;

        const Vector r_main = Vector(0,tank_z) - state.com;
        const Vector r_left = Vector(-tank_x, tank_z) - state.com;
        const Vector r_right = Vector(+tank_x, tank_z) - state.com;

        I += state.fuel_main * r_main.dot(r_main);
        I += state.fuel_left * r_left.dot(r_left);
        I += state.fuel_right * r_right.dot(r_right);

        if (I < 1.0f) I = 1.0f;
        return I;
    }


    Vector bodyCenterWorld() const {
        Vector com_offset_world = Vector::rot(state.com, state.angle);
        return state.pos - com_offset_world;
    }

    void computeLegWorldPositions(Vector& left, Vector& right) const
    {
        float half_leg_span = leg_spacing * 0.5f;
        float leg_local_z = -hull_height * 0.5f - landing_leg_length;

        Vector body_center = bodyCenterWorld();

        Vector left_local (-half_leg_span, leg_local_z);
        Vector right_local(+half_leg_span, leg_local_z);

        left = body_center + Vector::rot(left_local, state.angle);
        right = body_center + Vector::rot(right_local, state.angle);
    }

    Vector engineOffsetWorld(const Engine& e) const { //плечо в мировые
        Vector r_local = e.rel_pos - state.com;
        return Vector::rot(r_local, state.angle);
    }

    void solveSideCouple(float M_required) {
        engine_side_left.throttle_actual  = 0.0f;
        engine_side_right.throttle_actual = 0.0f;

        if (std::fabs(M_required) < 1e-6f) return;
        if (state.fuel_left <= 0.0f || state.fuel_right <= 0.0f) return;

        const Vector rL = engineOffsetWorld(engine_side_left); //плечи от цм
        const Vector rR = engineOffsetWorld(engine_side_right);
        const Vector rDiff = rL - rR;

        // ось X в локале
        const float baseL = 90.0f * Constants::DEG2RAD;
        const float baseR = -90.0f * Constants::DEG2RAD;

        const float dMax = 45.0f * Constants::DEG2RAD;
        const float minL = baseL - dMax;
        const float maxL = baseL + dMax;
        const float minR = baseR - dMax;
        const float maxR = baseR + dMax;

        const int N = 31; 
        float best_angL = baseL;
        float best_tau_per_T = 0.0f;

        for (int i = 0; i < N; ++i)
        {
            float u = (N == 1) ? 0.5f : (float)i / (float)(N - 1);
            float angL = minL + (maxL - minL) * u;

            Vector dir_local = Vector::fromAngle(angL); 
            Vector dir_world = Vector::rot(dir_local, state.angle).norm();

            //  M=(rL-rR)*(T * dir_world)
            float tau_per_T = rDiff.cross(dir_world); //момент на единицу тяги
            if (tau_per_T * M_required <= 0.0f) continue;

            if (std::fabs(tau_per_T) > std::fabs(best_tau_per_T)){//для мин тяги
                best_tau_per_T = tau_per_T;
                best_angL = angL;
            }
        }

        if (std::fabs(best_tau_per_T) < 1e-6f) return;

        float T_required = std::fabs(M_required / best_tau_per_T);

        // Ограничение по тяге каждого бокового
        T_required = std::max(0.0f, std::min(T_required, engine_side_left.max_thrust));

        float thr = (T_required / engine_side_left.max_thrust) * 100.0f;
        thr = std::max(0.0f, std::min(thr, 100.0f));
        float angR = best_angL - Constants::PI;

        if (angR < minR || angR > maxR) {
            float angR2 = angR + 2.0f * Constants::PI;
            if (angR2 < minR || angR2 > maxR){
                return;
            }
            angR = angR2;
        }

        engine_side_left.throttle_actual  = thr;
        engine_side_right.throttle_actual = thr;
        engine_side_left.dir_angle  = best_angL;
        engine_side_right.dir_angle = angR;
    }

public: 
    float main_engine_angle = 0.0f; // (рад) локально

    Lander() {
        float z_main = -hull_height * 0.5f;
        float dx_main = hull_width * 0.3f;

        engine_main_left = Engine(Vector(-dx_main, z_main), 0.0f, Constants::MAX_THRUST_MAIN);
        engine_main_right = Engine(Vector(+dx_main, z_main), 0.0f, Constants::MAX_THRUST_MAIN);

        float dx_side = hull_width * 0.6f;
        engine_side_left = Engine(Vector(-dx_side, 0.0f), 0.0f, Constants::MAX_THRUST_SIDE);
        engine_side_right = Engine(Vector(+dx_side, 0.0f), 0.0f, Constants::MAX_THRUST_SIDE);
        bindEngines();

        updateCOM();
    }
    Lander(const Lander& other) { *this = other; }

    Lander& operator=(const Lander& other) {
        if (this == &other) return *this;

        state = other.state;

        hull_width = other.hull_width;
        hull_height = other.hull_height;
        landing_leg_length = other.landing_leg_length;
        leg_spacing = other.leg_spacing;

        engine_main_left  = other.engine_main_left;
        engine_main_right = other.engine_main_right;
        engine_side_left  = other.engine_side_left;
        engine_side_right = other.engine_side_right;

        inertia = other.inertia;
        main_engine_angle = other.main_engine_angle;

        bindEngines();  
        return *this;
    }

    const LanderState& getState() const { return state; }
    LanderState& stateRef() { return state; }

    float getBodyWidth() const { return hull_width; }
    float getBodyHeight() const { return hull_height; }
    float getLegLength() const { return landing_leg_length; }
    float getLegSpacing() const { return leg_spacing; }

    Vector getBodyCenterWorld() const { return bodyCenterWorld(); }

    void getLegWorldPositions(Vector& left, Vector& right) const {
        computeLegWorldPositions(left, right);
    }

    int getEngineCount() const { return 4; }

    float getFootClearance() const {
        return state.com.y + getLegLength() + getBodyHeight() * 0.5f;
    }

    const Engine& getEngine(int i) const {
        if (i < 0 || i >= 4)
            return engine_main_left; 
        return *engines[i];
    }

    void setMainEngineAngleDeg(float deg) {
        main_engine_angle = deg * Constants::DEG2RAD;
    }

    void setTargetAngleDeg(float deg) {
        state.target_angle = deg * Constants::DEG2RAD;
    }

    void setMainThrottle(float p) {
        p = std::max(0.0f, std::min(p, 100.0f));
        engine_main_left.throttle_cmd  = p;
        engine_main_right.throttle_cmd = p;

    }

    void setSideThrottle(float L, float R) {
        L = std::max(0.0f, std::min(L, 100.0f));
        R = std::max(0.0f, std::min(R, 100.0f));
        engine_side_left.throttle_cmd  = L;
        engine_side_right.throttle_cmd = R;

    }

    float getEngineThrottle(int i) const {
        if (i < 0 || i >= 4)
            return 0.0f;
        return engines[i]->throttle_actual;
    }

    void setEngineThrottle(int i, float v) {
        if (i < 0 || i >= 4)
            return;

        v = std::max(0.0f, std::min(v, 100.0f));
        engines[i]->throttle_cmd = v;
    }

    void update(float dt) {
        for (int i = 0; i < 4; ++i)
            engines[i]->throttle_actual = engines[i]->throttle_cmd;
        updateCOM();
        inertia = computeInertia();

        float mass = state.getTotalMass();
        if (mass < Constants::DRY_MASS) mass = Constants::DRY_MASS;

        float angle_error = state.target_angle - state.angle;
        // [-pi; pi]
        while (angle_error > Constants::PI) angle_error -= 2.0f * Constants::PI;
        while (angle_error < -Constants::PI) angle_error += 2.0f * Constants::PI;

        float alpha_cmd = Constants::ANGLE_KP * angle_error - Constants::ANGLE_KD * state.omega;
        const float ALPHA_MAX = 4.0f; // рад/с^2
        alpha_cmd = std::clamp(alpha_cmd, -ALPHA_MAX, +ALPHA_MAX);

        float M_cmd = alpha_cmd * inertia;
        constexpr float MAX_ANG_ACCEL = 3.0f; // рад/с^2
        float M_limit = inertia * MAX_ANG_ACCEL;
        M_cmd = std::max(-M_limit,std::min(M_cmd, M_limit));
    
        solveSideCouple(M_cmd);
        engine_main_left.dir_angle  = main_engine_angle;
        engine_main_right.dir_angle = main_engine_angle;

        const float denom = (Constants::ISP * Constants::G0); //топливо ограничение

        auto thrustN = [](const Engine& e) {
            return e.max_thrust * (e.throttle_actual / 100.0f);
        };

        {
            float Tml = thrustN(engine_main_left);
            float Tmr = thrustN(engine_main_right);
            float burn_need = ((Tml + Tmr) / denom) * dt;
            if (burn_need > state.fuel_main && burn_need > 1e-8f) {
                float scale = state.fuel_main / burn_need;
                scale = std::max(0.0f, std::min(scale, 1.0f));
                engine_main_left.throttle_actual *= scale;
                engine_main_right.throttle_actual *= scale;
            }
        }

        {
            float Tsl = thrustN(engine_side_left);
            float burn_need = (Tsl / denom) * dt;
            if (burn_need > state.fuel_left && burn_need > 1e-8f)
            {
                float scale = state.fuel_left / burn_need;
                scale = std::max(0.0f, std::min(scale, 1.0f));

                engine_side_left.throttle_actual *= scale;
            }
        }
        {
            float Tsr = thrustN(engine_side_right);
            float burn_need = (Tsr / denom) * dt;
            if (burn_need > state.fuel_right && burn_need > 1e-8f)
            {
                float scale = state.fuel_right / burn_need;
                scale = std::max(0.0f, std::min(scale, 1.0f));
                engine_side_right.throttle_actual *= scale;
            }
        }

        
        Vector F_total(0,0);
        float  M_total = 0.0f;

        auto accumulate = [&](Engine& e)
        {
            Vector F = e.getForce(state);
            float M = e.getTorque(state);
            F_total += F;
            M_total += M;
        };

        accumulate(engine_main_left);
        accumulate(engine_main_right);
        accumulate(engine_side_left);
        accumulate(engine_side_right);

        ////////////////////////////////////
        F_total += Constants::Gravity() * mass;

        state.a = F_total / mass;
        state.v += state.a * dt;
        state.pos += state.v * dt;

        float angular_acc = M_total / inertia;
        state.omega += angular_acc * dt;
        state.angle += state.omega * dt;

        while (state.angle > Constants::PI)
            state.angle -= 2.0f * Constants::PI;
        while (state.angle < -Constants::PI)
            state.angle += 2.0f * Constants::PI;

        const float Tml = thrustN(engine_main_left);
        const float Tmr = thrustN(engine_main_right);
        const float Tsl = thrustN(engine_side_left);
        const float Tsr = thrustN(engine_side_right);
        const float burn_main = ((Tml + Tmr) / denom) * dt;
        const float burn_left = (Tsl / denom) * dt;
        const float burn_right = (Tsr / denom) * dt;

        state.fuel_main = std::max(0.0f, state.fuel_main - burn_main);
        state.fuel_left = std::max(0.0f, state.fuel_left - burn_left);
        state.fuel_right = std::max(0.0f, state.fuel_right - burn_right);

        if (state.fuel_main <= 0.0f) {
            engine_main_left.throttle_actual = 0.0f;
            engine_main_right.throttle_actual = 0.0f;
        }
        if (state.fuel_left <= 0.0f) {
            engine_side_left.throttle_actual = 0.0f;
        }
        if (state.fuel_right <= 0.0f) {
            engine_side_right.throttle_actual = 0.0f;
        }

        state.t += dt;

        updateCOM();
    }
};

#endif
