#include "State.h"
#include "Event.h"

Event::Event(double time)
    : time_(time)
{}

void Event::process(State & state)
{
    // Time update
    state.predict(time_);

    // Event-specific implementation
    update(state);
}
