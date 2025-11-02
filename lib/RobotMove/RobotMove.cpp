#include "RobotMove.hpp"

namespace RobotMove{

    void setIsModuleDebugModeActive(boolean isDebugModeActive){
        isModuleDebugModeActive = isDebugModeActive;
    }

    void calculateVelocityCouplingMatrix(){
        D[0][0] = -sin(phi);
        D[1][0] = -sin(phi);
        D[2][0] = sin(theta);
        D[3][0] = sin(theta);

        D[0][1] = cos(phi);
        D[1][1] = -cos(phi);
        D[2][1] = -cos(theta);
        D[3][1] = cos(theta);

        D[0][2] = 1;
        D[1][2] = 1;
        D[2][2] = 1;
        D[3][2] = 1;
    }

    void setup(boolean isDebugModeActive){

        setIsModuleDebugModeActive(isDebugModeActive);

        calculateVelocityCouplingMatrix();

        Utils::printMessageSetupDebug(isModuleDebugModeActive, moduleName);
    }

    float* calculateWheelVelocities(){
        float* wheelVelocities = new float[4];
        robotVelocities[2] = R*robotVelocities[2];

        for(int i=0; i<4; i++){
            for(int j=0; j<3; j++){
                wheelVelocities[i] = D[i][j]*robotVelocities[j];
            }
        }

        Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Velocidades do robô", robotVelocities[0], robotVelocities[1], robotVelocities[2]);
        Utils::printMessageLoopDebug(isModuleDebugModeActive, moduleName, "Velocidades das rodas", wheelVelocities[0], wheelVelocities[1], wheelVelocities[2], wheelVelocities[3]);
        return wheelVelocities;
    }

    void setRobotVelocities(float linearVelocity_x,float linearVelocity_y, float angularVelocity){
        robotVelocities[0] = linearVelocity_x;
        robotVelocities[1] = linearVelocity_y;
        robotVelocities[2] = angularVelocity;
    }
}