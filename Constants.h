#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Vector.h"

class Constants {
public:
    static Vector Gravity() {
        return Vector(0.0f, -1.62f);
    }
    static constexpr float DT = 0.01f;   

    static constexpr float HULL_WIDTH = 4.0f;   
    static constexpr float HULL_HEIGHT = 5.0f;
    static constexpr float LEG_LENGTH = 2.5f;   
    static constexpr float LEG_SPACING = 4.0f;
    static constexpr float DRY_MASS = 1500.0f; 
    static constexpr float FUEL_MAIN = 6000.0f; 
    static constexpr float FUEL_SIDE = 600.0f;  

    static constexpr float MAX_THRUST_MAIN = 25000.0f; 
    static constexpr float MAX_THRUST_SIDE = 5000.0f;   

   
    static constexpr float ISP = 311.0f;      // сек
    static constexpr float G0 = 9.80665f;  
    static constexpr float HULL_INERTIA = (Constants::DRY_MASS * (Constants::HULL_WIDTH  * Constants::HULL_WIDTH + Constants::HULL_HEIGHT * Constants::HULL_HEIGHT)) / 12.0f;

    static constexpr float ANGLE_KP = 0.8f;
    static constexpr float ANGLE_KD = 0.5f;

    static constexpr float MAX_LANDING_VZ = 2.5f;
    static constexpr float MAX_LANDING_VX = 1.5f;
    static constexpr float MAX_LANDING_ANGLE = 20.0f;    
    static constexpr float MAX_SAFE_SLOPE = 15.0f;    

    
    static constexpr float WORLD_WIDTH  = 350.0f;
    static constexpr float WORLD_HEIGHT = 350.0f;

    static constexpr float EPS = 0.01f;

    static constexpr float PI = 3.14159265359f;
    static constexpr float DEG2RAD = PI / 180.0f;
    static constexpr float RAD2DEG = 180.0f / PI;
};
#endif