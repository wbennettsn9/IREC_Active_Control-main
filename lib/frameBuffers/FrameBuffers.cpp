#include "FrameBuffers.h"

FrameBuffer* FrameBuffer::fetch(unsigned int n){
    if(n>maxFrameDepth){
        return nullptr;
    }
    FrameBuffer* head = this;
    // This could be done recursively, but this is safer
    for(int i = 0; i < n; i++){
        head = head->buffer;
        if(!head){
            // We have reached a null value
            // return null to avoid undefined behavior
            return nullptr;
        }
    }
    return head;
}

bool FrameBuffer::link(FrameBuffer* prev){
    if(!this->maxFrameDepth || !prev){
        return false;
    }
    this->buffer = prev;
    delete (this->fetch(maxFrameDepth));
    return true;
}

// when I care enough I will make this use generics to not reuse code but for now this works
Bframe* Bframe::getPrevious(unsigned int n){
    FrameBuffer* prev = this->fetch(n);
    if(!this){
        return nullptr;
    }
    return static_cast<Bframe*>(this->fetch(n));
}


Aframe* Aframe::getPrevious(unsigned int n){
    FrameBuffer* prev = this->fetch(n);
    if(!this){
        return nullptr;
    }
    return static_cast<Aframe*>(this->fetch(n));
}