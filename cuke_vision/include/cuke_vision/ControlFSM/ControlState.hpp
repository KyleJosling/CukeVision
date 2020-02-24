#pragma once
#include "controlFSMNode.hpp"

class controlFSM;

class ControlState {

    public:
        virtual void enter(controlFSM *control) = 0;
        virtual void toggle(controlFSM *control) = 0;
        virtual void exit(controlFSM *control) = 0;
        virtual ~ControlState() {}
};
