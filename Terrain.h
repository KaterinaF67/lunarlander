#ifndef TERRAIN_H
#define TERRAIN_H

#include <vector>
#include <algorithm>
#include <cmath>

#include "Vector.h"

class Point {
public:
    float x, z;
    Point(float x=0, float z=0) : x(x), z(z) {}
};

class LandingZone {
public:
    float x_start = 0;
    float x_end = 0;
    float avg_height = 0;
    float max_slope = 0;
    float safety_score = 0;
};

class Terrain {
private:
    std::vector<Point> surface;

    static inline float cross2(float ax, float ay, float bx, float by) {
        return ax * by - ay * bx;
    }

   
    static bool raySeg(const Vector& O, const Vector& D, const Vector& A, const Vector& B, float& t_out, Vector& P_out){
        Vector E = B - A;
        float denom = cross2(D.x, D.y, E.x, E.y);
        if (std::fabs(denom) < 1e-7f) return false; 

        Vector AO = A - O;
        float t = cross2(AO.x, AO.y, E.x, E.y) / denom;
        float u = cross2(AO.x, AO.y, D.x, D.y) / denom;

        if (t < 0.0f) return false;
        if (u < 0.0f || u > 1.0f) return false;

        P_out = O + D * t;
        t_out = t;
        return true;
    }

public:

    Terrain() = default;

  
    bool IntersectRay(Vector origin, Vector dir, Vector& hitPoint) const {
        if (surface.size() < 2) return false;

        Vector D = dir.norm();
        if (D.isZero()) return false;

        bool hit = false;
        float bestT = 1e30f;
        Vector bestP;

        for (size_t i = 0; i + 1 < surface.size(); ++i) {
            const Point& p0 = surface[i];
            const Point& p1 = surface[i + 1];
            Vector A(p0.x, p0.z);
            Vector B(p1.x, p1.z);

            float t;
            Vector P;
            if (!raySeg(origin, D, A, B, t, P)) continue;
            if (t < bestT) {
                bestT = t;
                bestP = P;
                hit = true;
            }
        }

        if (!hit) return false;
        hitPoint = bestP;
        return true;
    }

    void Clear() { surface.clear(); }

    void AddPoint(float x, float z){
        surface.emplace_back(x, z);
        std::sort(surface.begin(), surface.end(), [](const Point& a, const Point& b){ return a.x < b.x; });
    }

    int GetPointCount() const { return (int)surface.size(); }

    Point GetPoint(int i) const {
        if (i >= 0 && i < (int)surface.size()) return surface[i];
        return Point();
    }

   
    float GetHeightAt(float x) const{
        if (surface.empty()) return 0.0f;
        if (surface.size() == 1) return surface[0].z;

        if (x <= surface[0].x) return surface[0].z;
        if (x >= surface.back().x) return surface.back().z;

        for (size_t i = 0; i + 1 < surface.size(); ++i){
            const Point& p1 = surface[i];
            const Point& p2 = surface[i+1];

            if (x >= p1.x && x <= p2.x){
                float dx = p2.x - p1.x;
                if (dx < 1e-5f) return p1.z;

                float t = (x - p1.x) / dx;
                return p1.z + t * (p2.z - p1.z);
            }
        }
        return surface.back().z;
    }
 

    Vector GetNormalAt(float x) const {
        const float dx = 2.5f;
        float z1 = GetHeightAt(x - dx);
        float z2 = GetHeightAt(x + dx);
        float dzdx = (z2 - z1) / (2.0f * dx);

        Vector n(-dzdx, 1.0f);
        return n.norm();
    }

   
    float GetMinHeightInRange(float x1, float x2) const{
        float min_z = +1e9f;

        for (const auto& p : surface){
            if (p.x >= x1 && p.x <= x2) min_z = std::min(min_z, p.z);
        }

        if (min_z == +1e9f) {
            float xm = 0.5f * (x1 + x2);
            return GetHeightAt(xm);
        }
        return min_z;
    }

    float GetSlopeAt(float x) const{
        float dx = 5.0f;
        float z1 = GetHeightAt(x - dx);
        float z2 = GetHeightAt(x + dx);

        float slope = std::atan2((z2 - z1), 2.0f * dx);
        return std::fabs(slope * 180.0f / 3.14159265359f);
    }

    bool IsInSafeZone(float x, float slope_threshold) const{
        return GetSlopeAt(x) < slope_threshold;
    }

    bool IsBelowTerrain(float x, float z) const{
        return z <= GetHeightAt(x);
    }

    bool CheckCollisionPolygon(const std::vector<Point>& pts) const{
        for (auto& p : pts){
            if (IsBelowTerrain(p.x, p.z)) return true;
        }
        return false;
    }

  
    std::vector<LandingZone> FindSafeLandingZones(float world_width) const {
        std::vector<LandingZone> zones;
        if (surface.size() < 2) return zones;

        constexpr float MIN_ZONE_WIDTH = 20.0f;
        constexpr float THRESHOLD_SLOPE = 8.0f;
        constexpr float STEP = 1.0f;

        float x_min = surface.front().x;
        float x_max = surface.back().x;

        bool in_zone = false;
        float zone_start = 0;

        for (float x = x_min; x <= x_max; x += STEP){
            float slope = GetSlopeAt(x);

            if (slope < THRESHOLD_SLOPE){
                if (!in_zone){
                    in_zone = true;
                    zone_start = x;
                }
            }
            else{
                if (in_zone){
                    float x_end = x;
                    float w = x_end - zone_start;

                    if (w >= MIN_ZONE_WIDTH){
                        LandingZone z;
                        z.x_start = zone_start;
                        z.x_end = x_end;

                        // оценка высоты/наклона внутри зоны
                        const float mid = 0.5f * (zone_start + x_end);
                        z.avg_height = GetHeightAt(mid);

                        float maxSlope = 0.0f;
                        for (float xs = zone_start; xs <= x_end; xs += 1.0f) maxSlope = std::max(maxSlope, GetSlopeAt(xs));
                        z.max_slope = maxSlope;

                        // ширина + близость + плоскость
                        const float world_center = world_width * 0.5f;
                        const float distPenalty = std::fabs(mid - world_center) * 0.02f;
                        const float widthBonus = std::min(50.0f, w * 0.8f);
                        const float slopePenalty = maxSlope * 2.0f;
                        z.safety_score = 100.0f + widthBonus - distPenalty - slopePenalty;

                        zones.push_back(z);
                    }
                    in_zone = false;
                }
            }
        }

        std::sort(zones.begin(), zones.end(), [](const LandingZone& a, const LandingZone& b) { return a.safety_score > b.safety_score; });
        if (in_zone) {
            float x_end = x_max;
            float w = x_end - zone_start;
            if (w >= MIN_ZONE_WIDTH) {
                LandingZone z;
                z.x_start = zone_start;
                z.x_end  = x_end;
                const float mid = 0.5f *(zone_start + x_end);
                z.avg_height = GetHeightAt(mid);

                float maxSlope = 0.0f;
                for (float xs = zone_start; xs <= x_end; xs += 1.0f) maxSlope = std::max(maxSlope, GetSlopeAt(xs));
                z.max_slope = maxSlope;

                const float world_center = world_width * 0.5f;
                const float distPenalty = std::fabs(mid - world_center) * 0.02f;
                const float widthBonus = std::min(30.0f, w * 0.5f);
                const float slopePenalty = maxSlope * 2.0f;
                z.safety_score = 100.0f + widthBonus - distPenalty - slopePenalty;

                zones.push_back(z);
            }
        }
        return zones;
    }
};

#endif
