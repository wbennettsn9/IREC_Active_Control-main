#pragma once
#include "Vmath.h"

/*
Frame buffers are objects that store the time history of a certain object
They need to be linked with another framebuffer in order to work
*/
class FrameBuffer{
    public:
        using DestroyHook = void (*)(const FrameBuffer& iWillDie);
        
        FrameBuffer(unsigned long time, unsigned int depth = 0, DestroyHook hook = nullptr){
            maxFrameDepth = depth;
            onDestroy = hook;
            timestamp = time;
            buffer = nullptr;
        }
        // in case of initialization
        unsigned int maxFrameDepth = 0;
        unsigned long timestamp = 0;
        DestroyHook onDestroy = nullptr;

        virtual ~FrameBuffer(){
            if (onDestroy) onDestroy(*this);
        }

        // links the previous frame to a buffer
        bool link(FrameBuffer* prev);
        // Get Nth previous frame
    protected:
        FrameBuffer *buffer = nullptr;
        FrameBuffer* fetch(unsigned int n);
};

/*
Body Attached Reference Frame
*/
class Bframe : public FrameBuffer{
    public:
        Bframe(unsigned long time, unsigned int depth=0, DestroyHook hook = nullptr)
                : FrameBuffer(time, depth, hook) {}
        
        Bframe* getPrevious(unsigned int n = 1);
        
        Vec3 acceleration;
        Vec3 angular_velocity;
};

/*
Absolute frame
Location description given from some innertial reference frame
*/
class Aframe : public FrameBuffer{
    public:
        Aframe(unsigned long time, unsigned int depth=0, DestroyHook hook = nullptr)
            : FrameBuffer(time, depth, hook) {}

        Aframe* getPrevious(unsigned int n = 1);

        Vec3 acceleration;
        Vec3 velocity;
        Vec3 position;
        Vec3 angular_velocity;
        Vec3 orientation;
};

