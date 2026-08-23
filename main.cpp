#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

struct State {
    double x, y, z;       // position
    double vx, vy, vz;    // velocity
};

// force = gravity + drag
void getAcceleration(double vx, double vy, double vz, double mass, double Cd, double area, double rho, 
                     double &ax, double &ay, double &az) 
{
    // |v| 
    double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
    
    // drag magnitude
    double dragMagnitude = 0.5 * rho * Cd * area * speed * speed;

    // drag--->against motion
    double dragX = 0.0;
    double dragY = 0.0;
    double dragZ = 0.0;

    if (speed > 0.0) {
        dragX = -dragMagnitude * (vx / speed);
        dragY = -dragMagnitude * (vy / speed);
        dragZ = -dragMagnitude * (vz / speed);
    }

    // gravity
    double gravityY = -9.81 * mass;

    // second law of dynamic: a = F / m
    ax = dragX / mass;
    ay = (gravityY + dragY) / mass;
    az = dragZ / mass;
}

// RK4
State rk4Step(const State &current, double dt, double mass, double Cd, double area, double rho) 
{
    // k1
    double ax1, ay1, az1;
    getAcceleration(current.vx, current.vy, current.vz, mass, Cd, area, rho, ax1, ay1, az1);

    // k2
    double vx2 = current.vx + 0.5 * dt * ax1;
    double vy2 = current.vy + 0.5 * dt * ay1;
    double vz2 = current.vz + 0.5 * dt * az1;
    double ax2, ay2, az2;
    getAcceleration(vx2, vy2, vz2, mass, Cd, area, rho, ax2, ay2, az2);

    // k3
    double vx3 = current.vx + 0.5 * dt * ax2;
    double vy3 = current.vy + 0.5 * dt * ay2;
    double vz3 = current.vz + 0.5 * dt * az2;
    double ax3, ay3, az3;
    getAcceleration(vx3, vy3, vz3, mass, Cd, area, rho, ax3, ay3, az3);

    // k4
    double vx4 = current.vx + dt * ax3;
    double vy4 = current.vy + dt * ay3;
    double vz4 = current.vz + dt * az3;
    double ax4, ay4, az4;
    getAcceleration(vx4, vy4, vz4, mass, Cd, area, rho, ax4, ay4, az4);

    
    State next;
    
    // new position
    next.x = current.x + (dt / 6.0) * (current.vx + 2.0 * vx2 + 2.0 * vx3 + vx4);
    next.y = current.y + (dt / 6.0) * (current.vy + 2.0 * vy2 + 2.0 * vy3 + vy4);
    next.z = current.z + (dt / 6.0) * (current.vz + 2.0 * vz2 + 2.0 * vz3 + vz4);

    // new velocity
    next.vx = current.vx + (dt / 6.0) * (ax1 + 2.0 * ax2 + 2.0 * ax3 + ax4);
    next.vy = current.vy + (dt / 6.0) * (ay1 + 2.0 * ay2 + 2.0 * ay3 + ay4);
    next.vz = current.vz + (dt / 6.0) * (az1 + 2.0 * az2 + 2.0 * az3 + az4);

    return next;
}

int main() 
{
    // parameters
    double mass = 1.5;        // mass
    double Cd = 0.47;         // drag coefficent (assuming the vehicle is a sphere)
    double area = 0.01;       // cross-sectional area
    double rho = 1.225;       // air density
    
    double dt = 0.01;         // step
    double t = 0.0;           // initial time

    // initial state (launch at 45°)
    State state = {0.0, 0.0, 0.0, 30.0, 30.0, 0.0};

    
    std::ofstream outFile("trajectory_data.csv");
    outFile << "time,x,y,z,vx,vy,vz\n";

    // cycle until the object touch the ground
    while (state.y >= 0.0) 
    {
        
        outFile << t << "," << state.x << "," << state.y << "," << state.z << ","
                << state.vx << "," << state.vy << "," << state.vz << "\n";

        // next step
        state = rk4Step(state, dt, mass, Cd, area, rho);
        t += dt;
    }

    outFile.close();
    std::cout << "Simulation Completed\n";

    return 0;
}
