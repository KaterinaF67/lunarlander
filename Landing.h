#ifndef LANDING_H
#define LANDING_H

#include "Vector.h"
#include "Terrain.h"
#include "Constants.h"
#include "Lander.h"

#include <algorithm>
#include <cmath>
#include <vector>


enum class LandingStatus {
    Airborne, // ни одна нога не касается
    OneLegContact, // одна нога касается (зеленая зона)
    TwoLegsContact, // обе ноги касаются (зеленая зона)
    Landed,  // успешная посадка 
    Crashed // корпус касается земли / нога вне зоны / жёсткий контакт
};

struct LandingContact {
    bool left_touch = false;
    bool right_touch = false;
    bool hull_touch = false;

    bool left_in_zone = false;
    bool right_in_zone = false;

    float left_penetration = 0.0f;
    float right_penetration = 0.0f;
    float hull_penetration = 0.0f;

    float slope_deg = 0.0f;

    LandingStatus status = LandingStatus::Airborne;
};

class LandingSolver {
private:
    static Vector terrainNormalAt(const Terrain& terrain, float x) {
        const float dx = 2.5f;
        float z1 = terrain.GetHeightAt(x - dx);
        float z2 = terrain.GetHeightAt(x + dx);
        float dzdx = (z2 - z1) / (2.0f * dx);

        Vector n(-dzdx, 1.0f);
        float L = n.len();
        if (L < 1e-6f) return Vector(0, 1);
        return n / L;
    }

    static bool isInAnyZoneX(float x, const std::vector<LandingZone>& zones) { //зеленая зона
        for (const auto& z : zones) {
            if (x >= z.x_start && x <= z.x_end)
                return true;
        }
        return false;
    }

public:
    static LandingContact computeContact(const Lander& lander, const Terrain& terrain) {
        LandingContact C;
        constexpr float TOUCH_EPS = 0.02f;

        // мировые координаты ног 
        Vector L, R;
        lander.getLegWorldPositions(L, R);

        float zL = terrain.GetHeightAt(L.x);
        float zR = terrain.GetHeightAt(R.x);

        C.left_penetration = zL - L.y;
        C.right_penetration = zR - R.y;

        C.left_touch = (C.left_penetration> -TOUCH_EPS);
        C.right_touch = (C.right_penetration> -TOUCH_EPS);

        float half_w = lander.getBodyWidth()* 0.5f;
        float bottom_z = -lander.getBodyHeight()* 0.5f;

        Vector center = lander.getBodyCenterWorld();

        Vector C1 = center + Vector::rot(Vector(-half_w, bottom_z), lander.getState().angle);
        Vector C2 = center + Vector::rot(Vector(+half_w, bottom_z), lander.getState().angle);

        float p1 = terrain.GetHeightAt(C1.x) - C1.y;
        float p2 = terrain.GetHeightAt(C2.x) - C2.y;

        C.hull_penetration = std::max(p1, p2);
        C.hull_touch = (C.hull_penetration > -TOUCH_EPS);

        float mid_x = 0.5f * (L.x + R.x);
        C.slope_deg = terrain.GetSlopeAt(mid_x);

        // корпус
        if (C.hull_touch) {
            C.status = LandingStatus::Crashed;
            return C;
        }

        const auto zones = terrain.FindSafeLandingZones(Constants::WORLD_WIDTH);

        if (C.left_touch) C.left_in_zone = isInAnyZoneX(L.x, zones);
        if (C.right_touch) C.right_in_zone = isInAnyZoneX(R.x, zones);

        // касание ногой вне зоны = крах
        if (C.left_touch && !C.left_in_zone) {
            C.status = LandingStatus::Crashed;
            return C;
        }
        if (C.right_touch && !C.right_in_zone) {
            C.status = LandingStatus::Crashed;
            return C;
        }

        // статус контакта
        if (C.left_touch && C.right_touch) {
            // обе ноги касаются и обе в зоне 
            C.status = LandingStatus::TwoLegsContact;
        } else if (C.left_touch || C.right_touch) {
            // ровно одна нога касается и она в зоне
            C.status = LandingStatus::OneLegContact;
        } else {
            C.status = LandingStatus::Airborne;
        }

        return C;
    }

    static void resolve(Lander& lander, const LandingContact& C, const Terrain& terrain) {
        LanderState& S = lander.stateRef();

        if (C.status == LandingStatus::Airborne) return;

        constexpr float MU_ONE = 0.60f;
        constexpr float MU_TWO = 0.98f;
        constexpr float MU_CR = 0.95f;

        constexpr float OMEGA_DAMP_ONE = 0.55f;
        constexpr float OMEGA_DAMP_TWO = 0.15f;
        constexpr float OMEGA_DAMP_CR = 0.05f;

        auto applyContactImpulse = [&](const Vector& n, float mu, float omega_damp)
        {
            float vn = S.v.dot(n); // Проекция скорости на нормаль поверхности
            if (vn < 0.0f) S.v -= n * vn;

            Vector vt = S.v - n * S.v.dot(n); //касательная компонента скорости
            float vt_len = vt.len();

            if (vt_len < 0.20f) {
                S.v -= vt;
            } else {
                S.v -= vt * mu;
            }

            S.omega *= omega_damp; // гасим угловую скорость
        };

        Vector L, R; //мировые координаты ног
        lander.getLegWorldPositions(L, R);
        float nx = 0.5f * (L.x + R.x);
        if (C.status == LandingStatus::OneLegContact) {
            if (C.left_touch && !C.right_touch) nx = L.x;
            else if (C.right_touch && !C.left_touch) nx = R.x;
        }
        Vector n = terrainNormalAt(terrain, nx);

        if (C.status == LandingStatus::Crashed)
        {
            if (C.hull_penetration > 0.0f) {
                float push = std::min(C.hull_penetration + 0.03f, 0.25f);
                S.pos += n * push;
            }

            applyContactImpulse(n, MU_CR, OMEGA_DAMP_CR);
            S.v *= 0.15f;
            S.omega = 0.0f;
            return;
        }

        if (C.status == LandingStatus::OneLegContact)
        {
            float pen = std::max(C.left_penetration, C.right_penetration);
            if (pen > 0.0f) {
                float push = std::min(pen + 0.02f, 0.25f);
                S.pos += n * push;
            }

            applyContactImpulse(n, MU_ONE, OMEGA_DAMP_ONE);
            return;
        }

        if (C.status == LandingStatus::TwoLegsContact)
        {
            float pen = std::max(C.left_penetration, C.right_penetration);
            if (pen > 0.0f) {
                float push = std::min(pen + 0.02f, 0.25f);
                S.pos += n * push;
            }

            applyContactImpulse(n, MU_TWO, OMEGA_DAMP_TWO);

            if (std::fabs(S.v.x) < 2.0f * Constants::MAX_LANDING_VX) S.v.x = 0.0f;
            if (std::fabs(S.v.y) < 2.0f * Constants::MAX_LANDING_VZ) S.v.y = 0.0f;
            return;
        }
    }


    static LandingStatus getFinalStatus(const Lander& lander, const LandingContact& C) {
        const LanderState& S = lander.getState();

        if (C.status == LandingStatus::Crashed)
            return LandingStatus::Crashed;

        if (C.status == LandingStatus::OneLegContact || C.status == LandingStatus::TwoLegsContact) {
            const float vx_abs = std::fabs(S.v.x);
            const float vz_abs = std::fabs(S.v.y);
            const float ang_abs = std::fabs(S.angle * Constants::RAD2DEG);  
            if (vx_abs > 2.0f * Constants::MAX_LANDING_VX || vz_abs > 2.0f * Constants::MAX_LANDING_VZ || ang_abs > 1.5f * Constants::MAX_LANDING_ANGLE) {
                return LandingStatus::Crashed;
            }
            return C.status; // OneLegContact / TwoLegsContact
        }
        return C.status; // Airborne
    }
};

#endif
