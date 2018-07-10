#ifndef PILOT_H
#define PILOT_H
#include <string>

enum PilotMode
{
    PILOT_PASSIVE,
    PILOT_STABILIZED
};

class Pilot
{
private:
    PilotMode mode;
public:
    Pilot();
    void onState(bool connected, bool armed, bool guided, const std::string &mode);
};

#endif // PILOT_H
