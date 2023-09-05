#ifndef EVENT_H
#define EVENT_H

#include "State.h"

class Event
{
public:
    Event(double time);
    void process(State & state);

protected:
    virtual void update(State & state) = 0;
    double time_;    
};

#endif