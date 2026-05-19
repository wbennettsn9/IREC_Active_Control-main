#include "Vmath.h"
#include "FrameBuffers.h"

enum Physics{
    STATIONARY,
    BALLISTIC
};

class INS{
    private:
        Aframe current;
        Physics physicsMode = STATIONARY;
        unsigned int stationaryTimer = 0;
        void updatePhysicsMode(Bframe data);
    public:
        // Create an INS solver
        INS();
        // resets INS to default values
        void reset();
        // Perform Static Calibration on data stack
        Bframe generateStaticCalibration(Bframe data, bool updateSelf=true);
        // run the INS algorithm
        void update(Bframe data);
}