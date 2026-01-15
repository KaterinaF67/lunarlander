#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include <cmath>
#include <algorithm>

#include "DynamicArray.h"
#include "LazySequence.h"

#include "Lander.h"
#include "Terrain.h"
#include "Landing.h"
#include "Constants.h"


class RadarSample {
public:
    float t = 0.0f;
    float x = 0.0f;
    float z = 0.0f;
    float terrain_z = 0.0f;
    float altitude = 0.0f;

    float vx_est = 0.0f;
    float vz_est = 0.0f; 
};

class Autopilot {
private:
    Lander& lander;
    Terrain& terrain;

    bool has_red_zone = false;  //красная точка
    LandingZone red_zone{};
    float red_landing_x = 0.0f;

    bool  brake_target_locked = false;  //цель в фазе brake
    float brake_target_x = 0.0f;

    bool  vertical_target_locked = false;
    float vertical_target_x = 0.0f;
    float vertical_target_theta = 0.0f; // radians

public:
    struct RadarRay { //1 луч
        Vector origin;  //начало луча 
        Vector dir;   //длина 1
        bool hit = false;  //попал ли луч в рельеф
        Vector hitPoint; //точка пересечения луча с рельефом
        Vector endPoint;  //конец луча для отрисовки
        float verticalClearance = 1e9f; // origin.y - hitPoint.y 
        bool dangerous = false;
    };

    const DynamicArray<RadarRay>& GetRadarRays() const { return radar_rays; } //все лучи для графики
    bool GetRadarDanger() const { return radar_danger; } //хотя бы 1 флаг красный

private:
    static constexpr float SAFE_HEIGHT = 35.0f; //безопасная вертикаль
    static constexpr float MAX_VISUAL_LENGTH = 160.0f; //длина луча для графики если не пересек рельеф
    static constexpr int RADAR_RAY_COUNT = 7; //колво веерных лучей
    static constexpr float RADAR_HALF_ANGLE_DEG = 14.0f; //конус 28 градусов
    static constexpr float RADAR_MIN_SPEED = 1.0f;         

    DynamicArray<RadarRay> radar_rays;
    bool radar_danger = false;

    float radar_clearance_down = 1e9f;  // луч вниз
    float radar_clearance_target = 1e9f; // луч на цель
    bool  radar_target_hit = false;
    float radar_target_hit_y = -1e9f;


   
    void updateRadarRays(const LanderState& S) {
        radar_rays.clear();
        radar_danger = false;

        radar_clearance_down = 1e9f;  //под ногами
        radar_clearance_target = 1e9f;  // до цели
        radar_target_hit = false;  // пересек ли луч на цель рельеф
        radar_target_hit_y = -1e9f;  //высота земли в точке попадания

        const Vector origin = S.pos; // точка старта всех лучей - корабль

        Vector aimDir(0.0f, -1.0f); // по умолчанию радар вниз
        if (has_target_zone) {  //если есть цель смотрим по цели 
            const float zt = terrain.GetHeightAt(target_landing_x);
            Vector targetPoint(target_landing_x, zt);
            Vector toTarget = targetPoint - origin;  // вектор от корабля к точке посадки
            if (toTarget.len() > 1e-4f) {
                aimDir = toTarget.norm();
                if (aimDir.y > -1e-3f) aimDir = Vector(aimDir.x, -0.25f).norm();
            }
        } else if (S.v.len() >= RADAR_MIN_SPEED) { // нет цели смотрим по скорости
            aimDir = S.v.norm();
            if (aimDir.y > -1e-3f) aimDir = Vector(aimDir.x, -0.25f).norm(); //никогда не смотреть вверх
        }

        const float feetOffset = lander.getFootClearance(); // считать высоту будем для ног
        const float halfA = RADAR_HALF_ANGLE_DEG * Constants::DEG2RAD;

        for (int i = 0; i < RADAR_RAY_COUNT; ++i) {  //создаю веер лучей
            float u = (RADAR_RAY_COUNT == 1) ? 0.5f : float(i) / float(RADAR_RAY_COUNT - 1);
            float a = -halfA + 2.0f * halfA * u;

            Vector dir = Vector::rot(aimDir, a).norm();
            if (dir.y >= -1e-4f) continue;

            RadarRay ray;
            ray.origin = origin;
            ray.dir = dir;

            Vector hitPoint;
            ray.hit = terrain.IntersectRay(origin, dir, hitPoint); //попал ли луч в землю

            if (ray.hit) {
                ray.hitPoint = hitPoint;
                ray.endPoint = hitPoint;
                ray.verticalClearance = (origin.y - feetOffset) - hitPoint.y;
                ray.dangerous = (ray.verticalClearance < SAFE_HEIGHT);
                if (ray.dangerous) radar_danger = true;
            } else {
                ray.endPoint = origin + dir * MAX_VISUAL_LENGTH;
            }

            radar_rays.push_back(ray);
        }
        {// луч вниз
            RadarRay ray;
            ray.origin = origin;
            ray.dir = Vector(0.0f, -1.0f);

            Vector hitPoint;
            ray.hit = terrain.IntersectRay(origin, ray.dir, hitPoint);
            if (ray.hit) {
                ray.hitPoint = hitPoint;
                ray.endPoint = hitPoint;
                ray.verticalClearance = (origin.y - feetOffset) - hitPoint.y;
                radar_clearance_down = ray.verticalClearance;
                ray.dangerous = (ray.verticalClearance < SAFE_HEIGHT);
                if (ray.dangerous) radar_danger = true;
            } else {
                ray.endPoint = origin + ray.dir * MAX_VISUAL_LENGTH;
            }
            radar_rays.push_back(ray);
        }
        {//к цели
            RadarRay ray;
            ray.origin = origin;

            Vector dir_to_target(0.0f, -1.0f);
            if (has_target_zone) {
                const float zt = terrain.GetHeightAt(target_landing_x);
                Vector targetPoint(target_landing_x, zt);
                Vector toTarget = targetPoint - origin;
                if (toTarget.len() > 1e-4f) {
                    dir_to_target = toTarget.norm();
                    if (dir_to_target.y > -1e-3f) dir_to_target = Vector(dir_to_target.x, -0.25f).norm();
                }
            } else if (S.v.len() >= RADAR_MIN_SPEED) {
                dir_to_target = S.v.norm();
                if (dir_to_target.y > -1e-3f) dir_to_target = Vector(dir_to_target.x, -0.25f).norm();
            }

            ray.dir = dir_to_target;

            Vector hitPoint;
            ray.hit = terrain.IntersectRay(origin, ray.dir, hitPoint);

            if (ray.hit) {
                ray.hitPoint = hitPoint;
                ray.endPoint = hitPoint;
                ray.verticalClearance = (origin.y - feetOffset) - hitPoint.y;
                radar_target_hit = true;
                radar_target_hit_y = hitPoint.y;
                radar_clearance_target = ray.verticalClearance;

                ray.dangerous = (ray.verticalClearance < SAFE_HEIGHT);
                if (ray.dangerous) radar_danger = true;
            } else {
                ray.endPoint = origin + ray.dir * MAX_VISUAL_LENGTH;
            }
            radar_rays.push_back(ray);
        }
    }

    bool has_target_zone = false;
    LandingZone target_zone{};  //границы, тип — зелёная/красная
    float target_landing_x = 0.0f;
    bool has_z_target = false;  //PD false, true - жесткая цель
    float z_target = 0.0f;  
    static constexpr float Z_CLEARANCE = 30.0f;  //какую высоту держать
	static constexpr float EXTRA_MARGIN = 7.0f;  // для цели фазы 2

    enum class Phase { POLY, BRAKE, VERTICAL };
    Phase phase = Phase::POLY;
    Phase prev_phase = Phase::POLY;

    bool  has_radar_prev_clearance = false;
    float radar_prev_clearance = 0.0f;
    float radar_approach_speed = 0.0f; 

    bool  has_vz_filt = false;
    float vz_filt = 0.0f; // исправленная скорость

    
    static constexpr float KP_X_POLY = 0.14f;
    static constexpr float KD_X_POLY = 0.45f;

    static constexpr float KP_X_BRK = 0.08f;
    static constexpr float KD_X_BRK = 0.35f;

    static constexpr float KP_VZ = 3.0f;
    static constexpr float KD_VZ = 0.9f;
    static constexpr float KP_VZ_FINAL = 1.8;

    static constexpr float MAX_TILT_DEG_POLY = 22.0f;
    static constexpr float MAX_TILT_DEG_BRAKE = 18.0f;
    static constexpr float MAX_TILT_RATE_DEG_S = 30.0f;

    static constexpr float MAX_GROUND_ALIGN_DEG = 15.0f;

    static constexpr float X_LOCK = 5.0f;  
    static constexpr float VX_LOCK = 1.0f;  
	static constexpr float VY_LOCK = 2.7f;  
    const float Z_LOCK = 33.0f;
    static constexpr float BRAKE_PAD = 25.0f;   
    static constexpr float BRAKE_X_GATE = 15.0f;
    static constexpr float T_MIN = 10.0f;
    static constexpr float T_MAX = 50.0f;

    static constexpr float AX_USE_FRAC = 0.25f;

    static constexpr float H_EMERGENCY = 6.0f;    
    static constexpr float VZ_EMERGENCY = 0.5f;   

    float last_theta_cmd = 0.0f;
    bool  has_last_theta_cmd = false;

    float last_vz_error = 0.0f;
    bool  has_last_vz_error = false;

    bool  poly_initialized = false;
    float poly_t0 = 0.0f;
    float poly_T = 0.0f;
    float poly_a[6] = {0,0,0,0,0,0};

    LazySequence<RadarSample> radar_sequence;
    RadarSample last_radar_sample{};
    bool has_last_radar_sample = false;
    
    float alt_filt_prev = 0.0f; //если быстрое изменение рельефа
    bool  has_alt_filt_prev = false;
    float last_sim_t = -1.0f;
    mutable int radar_index = 0;

    RadarSample getRadarSample() {
        return radar_sequence.Get(radar_index++);
    }

    static float clampf(float v, float lo, float hi) {
        return std::max(lo, std::min(v, hi));
    }

	float marginEachSide() const {
	    const float leg_span = lander.getLegSpacing();
	    return 0.5f * leg_span + EXTRA_MARGIN;
	}

	bool isInsideZone(float x, const LandingZone& z) const {
	    return (x >= z.x_start && x <= z.x_end);
	}

    float clampToSafeInterval(float x, const LandingZone& z) const {
	    const float m = marginEachSide();
	    const float safe_left = z.x_start + m;
        const float safe_right = z.x_end - m;
        if (safe_right > safe_left) {
            return std::clamp(x, safe_left, safe_right);
        }
        return 0.5f * (z.x_start + z.x_end);
    }

	bool isZoneWideEnough(const LandingZone& z) const {
	    const float m = marginEachSide();
	    const float min_zone_width = 2.0f * m;
	    return ((z.x_end - z.x_start) >= min_zone_width);
	}

	void lockBrakeTarget(const LanderState& S) { //в фазе 2 выбираем 1 раз точку посадки
	    if (brake_target_locked) return;

        auto zones = terrain.FindSafeLandingZones(Constants::WORLD_WIDTH); //нет зеленых - красная точка
	    if (zones.empty() || !has_red_zone) {
	        brake_target_x = target_landing_x;
            brake_target_locked = true;
	        return;
	    }
        for (const auto& z : zones) { //уже над зеленой
	        if (!isZoneWideEnough(z)) continue;
	        if (isInsideZone(S.pos.x, z)) {
	            brake_target_x = clampToSafeInterval(S.pos.x, z);
	            brake_target_locked = true;
                return;
	        }
	    }
	    bool found = false;
	    float best_d = 1e9f;
	    float best_x = red_landing_x;
	    for (const auto& z : zones) {
	        if (!isZoneWideEnough(z)) continue;
	        const float zx = clampToSafeInterval(S.pos.x, z);
	        const float d = std::fabs(zx - S.pos.x);
	        if (!found || d < best_d) {
	            found = true;
	            best_d = d;
                best_x = zx;
            }
	    }

	    brake_target_x = found ? best_x : red_landing_x;
	    brake_target_locked = true;
        const float zg = terrain.GetHeightAt(brake_target_x);
        z_target = zg + Z_CLEARANCE;
        has_z_target = true;
	}

	void lockVerticalTargetIfNeeded() {   //vertical цель + нормаль
        if (vertical_target_locked) return;
	    vertical_target_x = brake_target_locked ? brake_target_x : target_landing_x;
	    float theta = normalAngleAt(vertical_target_x);
	    const float max_align = MAX_GROUND_ALIGN_DEG * Constants::DEG2RAD;
	    vertical_target_theta = clampf(theta, -max_align, +max_align);
        vertical_target_locked = true;
	}

    void resetControllerState(const LanderState& S) {
        phase = Phase::POLY;
        poly_initialized = false;
        poly_t0 = S.t;
        poly_T = 0.0f;
        for (int i = 0; i < 6; ++i) poly_a[i] = 0.0f;

        radar_index = 0;
        has_last_radar_sample = false;
        last_radar_sample = RadarSample{};

        has_alt_filt_prev = false;
        alt_filt_prev = 0.0f;

        radar_danger = false;
        if (radar_rays.size() > 0) radar_rays.clear();

        vz_filt = 0.0f;
        has_last_vz_error = false;
        last_vz_error = 0.0f;

        has_last_theta_cmd = false;
        last_theta_cmd = 0.0f;

        last_sim_t = S.t;

	        has_red_zone = false;
	        red_zone = LandingZone{};
	        red_landing_x = 0.0f;

	        brake_target_locked = false;
	        brake_target_x = 0.0f;

	        vertical_target_locked = false;
	        vertical_target_x = 0.0f;
	        vertical_target_theta = 0.0f;
    }

    const char* GetPhaseName() const {
        switch (phase) {
            case Phase::POLY:return "POLY";
            case Phase::BRAKE:return "BRAKE";
            case Phase::VERTICAL: return "VERTICAL";
            default: return "UNKNOWN";
        }
    }


    void selectTargetZone() { //красная
        auto zones = terrain.FindSafeLandingZones(Constants::WORLD_WIDTH);
        if (zones.empty()) {
            has_target_zone = false;
            has_z_target = false;
            return;
        }

        const LanderState& S = lander.getState();
        const float lx = S.pos.x;

        const float leg_span = lander.getLegSpacing();
        const float extra_margin = EXTRA_MARGIN;
        const float margin_each_side = 0.5f * leg_span + extra_margin;
        const float min_zone_width = 2.0f * margin_each_side;

        bool found_any = false;
        float best_dist = 0.0f;
        LandingZone best_zone{};
        float best_target_x = lx;

        for (const auto& z : zones) {
            const float width = z.x_end - z.x_start;
            if (width < min_zone_width) continue;

            const float safe_left = z.x_start + margin_each_side;
            const float safe_right = z.x_end - margin_each_side;
            if (safe_right <= safe_left) continue;

            float candidate = std::clamp(lx, safe_left, safe_right);
            float dist = std::fabs(candidate - lx);

            if (!found_any || dist < best_dist) {
                found_any = true;
                best_dist = dist;
                best_zone = z;
                best_target_x = candidate;
            }
        }

        if (!found_any) {
            //ближайшая зона если нет подходящей
            bool fb_found = false;
            float fb_best = 0.0f;
            LandingZone fb_zone{};

            for (const auto& z : zones) {
                float nearest = std::clamp(lx, z.x_start, z.x_end);
                float dist = std::fabs(nearest - lx);

                if (!fb_found || dist < fb_best) {
                    fb_found = true;
                    fb_best = dist;
                    fb_zone = z;
                }
            }

            if (!fb_found) {
                has_target_zone = false;
                has_z_target = false;
                return;
            }

            best_zone = fb_zone;
            float safe_left = fb_zone.x_start + margin_each_side;
            float safe_right = fb_zone.x_end - margin_each_side;
            if (safe_right > safe_left) {
                best_target_x = std::clamp(lx, safe_left, safe_right);
            } else {
                best_target_x = 0.5f * (fb_zone.x_start + fb_zone.x_end);
            }
        }

        target_zone = best_zone;
        target_landing_x = best_target_x;
        has_target_zone = true;

	    
	    has_red_zone = true;
	    red_zone = best_zone;
	    red_landing_x = target_landing_x;

	    brake_target_locked = false;
	    vertical_target_locked = false;

        float z_ground_target = terrain.GetHeightAt(target_landing_x);
        z_target = z_ground_target + Z_CLEARANCE;
        has_z_target = true;

        phase = Phase::POLY;
        poly_initialized = false;
        has_last_theta_cmd = false;
        has_last_vz_error = false;
    }

    float estimateMaxAx(float mass, float tilt_deg) const {  //макс горизонт ускорение
        float T_total = 2.0f* Constants::MAX_THRUST_MAIN;
        float tilt_rad = tilt_deg* Constants::DEG2RAD;
        float ax_max = (T_total / mass)* std::sin(tilt_rad);
        if (ax_max < 0.5f) ax_max = 0.5f;
        return ax_max * AX_USE_FRAC;
    }


    float choosePolyTime(float dx, float vx, float mass) const {
        float dist = std::fabs(dx);
        float v0= std::fabs(vx);

        float ax_max = estimateMaxAx(mass, MAX_TILT_DEG_POLY);

        float t_brake = v0 / ax_max;
        float d_brake = 0.5f * v0 * v0 / ax_max;  //тормозной путь

        float T = 0.0f;
        if (dist > d_brake) {
            float d_rem = dist - d_brake;
            float v_mid = std::max(1.0f, 0.5f * v0);
            float t_cruise = d_rem / v_mid;
            T = 2.0f* t_brake + t_cruise;
        } else {
            T = 2.0f* t_brake;
        }
        return clampf(T, T_MIN, T_MAX);
    }

    
    static bool Solve3x3(float M[3][3], float R[3]) {
        for (int i = 0; i < 3; ++i) {
            int pivot = i;
            float maxv = std::fabs(M[i][i]);
            for (int j = i+1; j < 3; ++j) {
                float v = std::fabs(M[j][i]);
                if (v > maxv) { maxv = v; pivot = j; }
            }
            if (maxv < 1e-8f) return false;

            if (pivot != i) {
                for (int k = 0; k < 3; ++k) std::swap(M[i][k], M[pivot][k]);
                std::swap(R[i], R[pivot]);
            }

            float diag = M[i][i];
            for (int k = i; k < 3; ++k) M[i][k] /= diag;
            R[i]/= diag;

            for (int j = i+1; j < 3; ++j) {
                float f = M[j][i];
                for (int k = i; k < 3; ++k) M[j][k] -= f * M[i][k];
                R[j] -= f * R[i];
            }
        }

        for (int i = 2; i >= 0; --i) {
            for (int j = i+1; j < 3; ++j) R[i]-= M[i][j] * R[j];
        }
        return true;
    }

    void buildPolyQuintic(float x0, float v0, float x1, float T, float t0) {
        if (T < 1e-3f) T = 1e-3f;

        float a0 = x0;
        float a1 = v0;
        float a2 = 0.0f;

        float T2 = T*T;
        float T3 = T2*T;
        float T4 = T3*T;
        float T5 = T4*T;

        float M[3][3] = {
            {T3,     T4,      T5},
            {3*T2,   4*T3,    5*T4},
            {6*T,   12*T2,   20*T3}
        };

        float R[3] = {
            x1 - (a0 + a1*T + a2*T2),
            -(a1 + 2*a2*T),
            -(2*a2)
        };

        if (!Solve3x3(M, R)){
            poly_a[0]=x0; poly_a[1]=(x1-x0)/T; poly_a[2]=0;
            poly_a[3]=0; poly_a[4]=0; poly_a[5]=0;
        }
        else {
            poly_a[0]=a0; poly_a[1]=a1; poly_a[2]=a2;
            poly_a[3]=R[0]; poly_a[4]=R[1]; poly_a[5]=R[2];
        }

        poly_T = T;
        poly_t0 = t0;
        poly_initialized = true;
    }

    void polyReference(float t_now, float& x_ref, float& vx_ref, float& ax_ref) const {
        if (!poly_initialized || poly_T <= 0.0f) {
            x_ref = target_landing_x;
            vx_ref = 0.0f;
            ax_ref = 0.0f;
            return;
        }

        float t = t_now - poly_t0;
        t = clampf(t, 0.0f, poly_T);

        float t2=t*t, t3=t2*t, t4=t3*t, t5=t4*t;
        float a0=poly_a[0], a1=poly_a[1], a2=poly_a[2], a3=poly_a[3], a4=poly_a[4], a5=poly_a[5];

        x_ref  = a0 + a1*t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
        vx_ref = a1 + 2*a2*t + 3*a3*t2 + 4*a4*t3 + 5*a5*t4;
        ax_ref = 2*a2 + 6*a3*t + 12*a4*t2 + 20*a5*t3;
    }

   
    float vzTargetToZ(float h_to_target) const {
        float h = std::max(0.0f, h_to_target);

        if(h > 200.0f) return -10.0f;
        else if (h > 100.0f) return -8.0f;
        else if (h > 50.0f) return -6.0f;
        else if (h > 25.0f) return -5.0f;
        else if (h > 20.0f) return -4.0f;
        else if (h > 15.0f) return -3.0f;
        else if (h >10.0f) return -2.5f;
        else if (h >7.0f) return -2.0f;
        else if (h > 3.0f) return -1.0f;
        else if (h > 2.0f) return -0.5f;
        else return -1.5f;
    }

    float vzTargetTouchdown(float h_to_ground) const {
        float h = std::max(0.0f, h_to_ground);

        if (h > 20.0f) return -10.0f;
        if (h > 10.0f) return -5.0f;
        if (h > 5.0f) return -3.0f;

        if (h > 2.0f) {
            float t = (h - 2.0f) / 3.0f; // 0..1
            return (-1.0f) * (1.0f - t) + (-3.0f) * t;
        }

        return -0.30f;
    }

    float normalAngleAt(float x) const {
        const float dx = 2.5f;
        float z1 = terrain.GetHeightAt(x - dx);
        float z2 = terrain.GetHeightAt(x + dx);
        float dzdx = (z2 - z1) / (2.0f * dx);

        Vector n(-dzdx, 1.0f);
        float L = n.len();
        if (L < 1e-6f) return 0.0f;
        n = n / L;

        return std::atan2(-n.x, n.y);
    }

public:
Autopilot(Lander& l, Terrain& t) : lander(l), terrain(t),
      radar_sequence([this](int) mutable -> RadarSample {

          const LanderState& S = lander.getState();
          float x = S.pos.x;
          float z = S.pos.y;
          float terrain_z = terrain.GetHeightAt(x);

          RadarSample rs;
          rs.t = S.t;
          rs.x = x;
          rs.z = z;
          rs.terrain_z = terrain_z;

          // Сырая радарная высота 
          const float alt_raw = z - terrain_z;

          float dt_local = Constants::DT;
          if (has_last_radar_sample) {
              dt_local = rs.t - last_radar_sample.t;
              if (dt_local < 1e-5f)
                  dt_local = Constants::DT;
          }

          
          float alt_filt = alt_raw;
          if (!has_alt_filt_prev) {
              alt_filt = alt_raw;
              alt_filt_prev = alt_filt;
              has_alt_filt_prev = true;
          } else {
              const float vy_world = S.v.y;   
              const float eps = 0.6f;
              const float mass = std::max(S.getTotalMass(), Constants::DRY_MASS);
              const float a_max = (2.0f * Constants::MAX_THRUST_MAIN) / mass;
              const float a_up_phys =
                  std::max(0.0f, a_max + Constants::Gravity().y);

              const float max_up =
                  std::max(0.0f, vy_world) * dt_local +
                  0.5f * a_up_phys * dt_local * dt_local + eps;

              const float dh = alt_raw - alt_filt_prev;
              if (dh > max_up) {
                  alt_filt = alt_filt_prev + max_up;
              } else {
                  alt_filt = alt_raw;
              }
              alt_filt_prev = alt_filt;
          }

          rs.altitude = alt_filt;

          
          if (!has_last_radar_sample) {
              rs.vx_est = 0.0f;
              rs.vz_est = 0.0f;
          } else {
              rs.vx_est = (rs.x - last_radar_sample.x) / dt_local;

              
              float v_alt = (rs.altitude - last_radar_sample.altitude) / dt_local;

              //Скорость роста рельефа
              float v_terrain = (rs.terrain_z - last_radar_sample.terrain_z) / dt_local;
              
              rs.vz_est = v_alt + v_terrain;
          }

          last_radar_sample = rs;
          has_last_radar_sample = true;
          return rs;
      })
{}



	float GetTargetX() const {
	    if (phase == Phase::POLY) return has_red_zone ? red_landing_x : target_landing_x;
	    return brake_target_locked ? brake_target_x : target_landing_x;
	}
    const char* GetPhaseText() const { return GetPhaseName(); }

void Update(){
    const LanderState& S = lander.getState();
    if (last_sim_t >= 0.0f && (S.t + 1e-6f < last_sim_t || (S.t < 0.05f && last_sim_t > 0.2f))) {
        resetControllerState(S);
    }
    last_sim_t = S.t;

    RadarSample rs = getRadarSample();

    LandingContact contact = LandingSolver::computeContact(lander, terrain);
    LandingStatus status = LandingSolver::getFinalStatus(lander, contact);

    if (status == LandingStatus::Crashed || status == LandingStatus::Landed) {
        lander.setMainThrottle(0.0f);
        lander.setTargetAngleDeg(0.0f);
        return;
    }

    if (status == LandingStatus::OneLegContact) {
        lander.setMainThrottle(0.0f);
        lander.setTargetAngleDeg(S.angle * Constants::RAD2DEG);
        return;
    }

    if (status == LandingStatus::TwoLegsContact) {
        Vector LL, RR;
        lander.getLegWorldPositions(LL, RR);
        float cx = 0.5f * (LL.x + RR.x);

        float theta = normalAngleAt(cx);
        float max_align = MAX_GROUND_ALIGN_DEG * Constants::DEG2RAD;
        theta = clampf(theta, -max_align, +max_align);

        lander.setMainThrottle(0.0f);
        lander.setTargetAngleDeg(theta * Constants::RAD2DEG);
        return;
    }

    if (!has_target_zone || !has_z_target) {
        selectTargetZone();
        if (!has_target_zone) {
            target_landing_x = rs.x;
            has_target_zone = true;

            z_target = terrain.GetHeightAt(target_landing_x) + Z_CLEARANCE;
            has_z_target = true;
        }
    }

    const float dt = Constants::DT;

    // Фильтр оценки вертикальной скорости 
    if (!has_vz_filt) {
        vz_filt = rs.vz_est;
        has_vz_filt = true;
    } else {
        const float a = 0.2f;
        vz_filt = (1.0f - a) * vz_filt + a * rs.vz_est;
    }

    
    if (phase != prev_phase) {
        has_last_vz_error = false;
        has_last_theta_cmd = false;
        prev_phase = phase;
    }

    float mass = std::max(S.getTotalMass(), Constants::DRY_MASS);
    const float poly_target_x = has_red_zone ? red_landing_x : target_landing_x;
    float dx_poly = poly_target_x - rs.x;

    // POLY -> BRAKE 
    if (phase == Phase::POLY) {
        float ax_max = estimateMaxAx(mass, MAX_TILT_DEG_BRAKE);
        float d_stop = (rs.vx_est * rs.vx_est) / (2.0f * ax_max + 1e-6f);

        const bool over_target_x = (std::fabs(dx_poly) < BRAKE_X_GATE);
        const bool ready_by_dynamics = (std::fabs(dx_poly) <= d_stop + BRAKE_PAD);

        if (over_target_x && ready_by_dynamics) {
            phase = Phase::BRAKE;
            brake_target_locked = false;
            vertical_target_locked = false;
        }
    }

    if (phase == Phase::BRAKE) {
        lockBrakeTarget(S);
    }

   
    // BRAKE -> VERTICAL 
    if (phase == Phase::BRAKE) {

        //  над безопасным интервалом зоны так, что обе ноги помещаются
        auto overSafeInterval = [&]() -> bool {
            if (!has_target_zone) return false;
            const float m = marginEachSide();
            const float safe_left  = target_zone.x_start + m;
            const float safe_right = target_zone.x_end   - m;
            return (safe_right > safe_left) && (rs.x >= safe_left) && (rs.x <= safe_right);
        };

       
        const float dx_brake = brake_target_x - rs.x;

        if (
            rs.altitude < Z_LOCK &&
            (std::fabs(rs.vx_est) < VX_LOCK) &&
            (std::fabs(S.v.y) < VY_LOCK) &&
            ( overSafeInterval() || (std::fabs(dx_brake) < X_LOCK) )
        ) {
            phase = Phase::VERTICAL;
            has_last_vz_error = false;
            has_last_theta_cmd = false;
            lockVerticalTargetIfNeeded();
        }
    }


   
    float z_ground_target = terrain.GetHeightAt(target_landing_x);
    float h_to_target = S.pos.y - z_target;

    float vz_target = 0.0f;

    if (phase == Phase::VERTICAL) {
        vz_target = vzTargetTouchdown(rs.altitude);
        if (rs.altitude < H_EMERGENCY) {
            vz_target = std::max(vz_target, -VZ_EMERGENCY);
        }
    } else {
        vz_target = vzTargetToZ(h_to_target);
        if (rs.altitude < H_EMERGENCY) {
            vz_target = std::max(vz_target, -VZ_EMERGENCY);
        }
    }

   
    updateRadarRays(S);

    
    float h = 1e9f;
    for (int i = 0; i < radar_rays.size(); ++i) {
        const RadarRay& ray = radar_rays[i];
        if (!ray.hit) continue;
        if (ray.dir.y >= 0.0f) continue;
        if (ray.verticalClearance < h) h = ray.verticalClearance;
    }

    
    if (phase == Phase::POLY && h < SAFE_HEIGHT) {
        float alpha = (SAFE_HEIGHT - h) / SAFE_HEIGHT;
        alpha = clampf(alpha, 0.0f, 1.0f);

        static constexpr float VZ_MAX_UP = 5.0f; 
        float vz_up = alpha * VZ_MAX_UP;

       
        vz_target = std::max(vz_target, vz_up);
    }


  
    float az_cmd = 0.0f;

    if (phase == Phase::VERTICAL) {
        float vz_error = vz_target - vz_filt;
        az_cmd = KP_VZ_FINAL * vz_error;
    } else {
        float vz_error = vz_target - vz_filt;

        float dvz_err = 0.0f;
        if (has_last_vz_error) dvz_err = (vz_error - last_vz_error) / dt;
        last_vz_error = vz_error;
        has_last_vz_error = true;

        az_cmd = KP_VZ * vz_error + KD_VZ * dvz_err;
    }

    float az_total = az_cmd + Constants::Gravity().y;
    az_total = std::max(0.2f, az_total);

    float a_max = (2.0f * Constants::MAX_THRUST_MAIN) / mass;
    az_total = clampf(az_total, 0.0f, a_max);

    
    float ax_cmd = 0.0f;

    if (phase == Phase::VERTICAL) {
        ax_cmd = 0.0f;
    }
    else if (phase == Phase::BRAKE) {
        float ax_max = estimateMaxAx(mass, MAX_TILT_DEG_BRAKE);

        const float active_brake_target_x = brake_target_locked ? brake_target_x : target_landing_x;
        float x_err = active_brake_target_x - rs.x;
        float vx_err = 0.0f - rs.vx_est;

        ax_cmd = KP_X_BRK * x_err + KD_X_BRK * vx_err;
        ax_cmd = clampf(ax_cmd, -ax_max, +ax_max);
    }
    else { // POLY
        if (!poly_initialized) {
            float T = choosePolyTime(dx_poly, rs.vx_est, mass);
            buildPolyQuintic(rs.x, rs.vx_est, poly_target_x, T, S.t);
        }

        float x_ref, vx_ref, ax_ref;
        polyReference(S.t, x_ref, vx_ref, ax_ref);

        float x_err  = x_ref  - rs.x;
        float vx_err = vx_ref - rs.vx_est;

        ax_cmd = ax_ref + KP_X_POLY * x_err + KD_X_POLY * vx_err;

        float ax_max = estimateMaxAx(mass, MAX_TILT_DEG_POLY);
        ax_cmd = clampf(ax_cmd, -ax_max, +ax_max);

        float t_rel = S.t - poly_t0;
        if (t_rel >= poly_T - 1e-3f) {
            if (std::fabs(dx_poly) < BRAKE_X_GATE) {
                phase = Phase::BRAKE;
                brake_target_locked = false;
                vertical_target_locked = false;
            } else {
                poly_initialized = false;
            }
        }
    }

    // ax^2 + az^2 <= a_max^2
    {
        float az_used = az_total;
        float ax_limit = std::sqrt(std::max(0.0f, a_max*a_max - az_used*az_used));
        ax_cmd = clampf(ax_cmd, -ax_limit, +ax_limit);
    }

   
    float theta_cmd = 0.0f;

    if (phase == Phase::VERTICAL) {
        lockVerticalTargetIfNeeded();
        theta_cmd = vertical_target_theta;

        const float max_rate = MAX_TILT_RATE_DEG_S * Constants::DEG2RAD;
        const float max_step = max_rate * dt;
        if (!has_last_theta_cmd) {
            last_theta_cmd = theta_cmd;
            has_last_theta_cmd = true;
        } else {
            float dth = theta_cmd - last_theta_cmd;
            while (dth > Constants::PI) dth -= 2.0f * Constants::PI;
            while (dth < -Constants::PI) dth += 2.0f * Constants::PI;
            dth = clampf(dth, -max_step, +max_step);
            theta_cmd = last_theta_cmd + dth;
            last_theta_cmd = theta_cmd;
        }
    }
    else {
        float max_tilt_deg = (phase == Phase::POLY) ? MAX_TILT_DEG_POLY : MAX_TILT_DEG_BRAKE;
        float max_tilt = max_tilt_deg * Constants::DEG2RAD;

        theta_cmd = std::atan2(-ax_cmd, az_total);
        theta_cmd = clampf(theta_cmd, -max_tilt, +max_tilt);

        const float max_rate = MAX_TILT_RATE_DEG_S * Constants::DEG2RAD;
        const float max_step = max_rate * dt;

        if (!has_last_theta_cmd) {
            last_theta_cmd = theta_cmd;
            has_last_theta_cmd = true;
        }
        else {
            float dth = theta_cmd - last_theta_cmd;
            while (dth > Constants::PI) dth -= 2.0f * Constants::PI;
            while (dth < -Constants::PI) dth += 2.0f * Constants::PI;

            dth = clampf(dth, -max_step, +max_step);
            theta_cmd = last_theta_cmd + dth;
            last_theta_cmd = theta_cmd;
        }

        theta_cmd = clampf(theta_cmd, -max_tilt, +max_tilt);
    }

   
    float T_total_max = 2.0f * Constants::MAX_THRUST_MAIN;

    float cos_th = std::cos(theta_cmd);
    if (cos_th < 0.25f) cos_th = 0.25f;

    {
        float az_max = (T_total_max / mass) * cos_th;
        if (az_total > az_max) az_total = az_max;
    }

    float Fz_required = mass * az_total;
    float T_required  = Fz_required / cos_th;

    float main_throttle = (T_required / T_total_max) * 100.0f;
    main_throttle = clampf(main_throttle, 0.0f, 100.0f);

    lander.setMainThrottle(main_throttle);
    lander.setTargetAngleDeg(theta_cmd * Constants::RAD2DEG);
}


};

#endif
    