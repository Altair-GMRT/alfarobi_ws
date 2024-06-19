#ifndef ALFAROBI_H
#define ALFAROBI_H


namespace robotis_op
{

class Alfarobi
{
public:
    enum Soccer_Half
    {
        First_Half  = 1,
        Second_Half = 2,
    };

    enum Robot_Zone
    {
        Zone1      = 1,
        Zone2      = 2,
        Zone3      = 3,
        Zone4      = 4,
    };

    enum Robot_Status
    {
        Waited               = 0,
        TrackingAndFollowing = 1,
        ReadyToKick          = 2,
        ReadyToCeremony      = 3,
        ReadyToGetup         = 4,
    };

    enum Button
    {
        B1 = 1,
        B2 = 2,
        B3 = 3,
        B4 = 4,
        NoData = 999,
    };

    enum Controller
    {
        W = 1,
        A = 2,
        D = 3,
        Q = 4,
        E = 5,
        K = 6,
        S = 7,
        Z = 8,
        C = 9,
        NoDatas = 998,
    };

    Alfarobi()
    {
    }

    virtual ~Alfarobi()
    {
    }

    virtual void setModuleEnable()
    {
    }

    virtual void setModuleDisable()
    {
    }

protected:
    bool enable_;
};

} /* namespace robotis_op */

#endif
