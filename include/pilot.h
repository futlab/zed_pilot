#ifndef PILOT_H
#define PILOT_H
#include <string>
//#include <Eigen/Core>

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
    void onPose();
};

#endif // PILOT_H
