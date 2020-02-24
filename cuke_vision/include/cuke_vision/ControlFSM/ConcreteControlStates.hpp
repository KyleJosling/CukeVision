// ----------------------------------------------------------
// NAME: FSM Node State Header
// DESCRIPTION: This header file defines the control state
// classes.
// ----------------------------------------------------------


#include "ControlState.hpp"
#include "ControlFSM.hpp"

class Idle : public ControlState {
    
    public:
        void enter(ControlFSM *control) {}
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control) {}
        static ControlState &getInstance();

    private:
        Idle() {}
        Idle(const Idle &other);
        Idle & operator =(const Idle &other);


};

class StartUp : public ControlState {
    
    public:
        void enter(ControlFSM *control);
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control) {}
        static ControlState &getInstance();

    private:
        StartUp() {}
        StartUp(const StartUp &other);
        StartUp & operator =(const StartUp &other);


};

class ScanCucumbers : public ControlState {
    
    public:
        void enter(ControlFSM *control);
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control);
        static ControlState &getInstance();

    private:
        ScanCucumbers() {}
        ScanCucumbers(const ScanCucumbers &other);
        ScanCucumbers & operator =(const ScanCucumbers &other);

};

class SortCucumbers : public ControlState {
    
    public:
        void enter(ControlFSM *control);
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control) {}
        static ControlState &getInstance();

    private:
        SortCucumbers() {}
        SortCucumbers(const SortCucumbers &other);
        SortCucumbers & operator =(const SortCucumbers &other);

};

class SaveOccupancyMap : public ControlState {
    
    public:
        void enter(ControlFSM *control);
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control) {}
        static ControlState &getInstance();

    private:
        SaveOccupancyMap() {}
        SaveOccupancyMap(const SaveOccupancyMap &other);
        SaveOccupancyMap & operator =(const SaveOccupancyMap &other);

};

class MoveItStartUp : public ControlState {
    
    public:
        void enter(ControlFSM *control);
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control) {}
        static ControlState &getInstance();

    private:
        MoveItStartUp() {}
        MoveItStartUp(const MoveItStartUp &other);
        MoveItStartUp & operator =(const MoveItStartUp &other);

};

class ShutDown : public ControlState {
    
    public:
        void enter(ControlFSM *control);
        void toggle(ControlFSM *control);
        void exit(ControlFSM *control) {}
        static ControlState &getInstance();

    private:
        ShutDown() {}
        ShutDown(const ShutDown &other);
        ShutDown & operator =(const ShutDown &other);

};
