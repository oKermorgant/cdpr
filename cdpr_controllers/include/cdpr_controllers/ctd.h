#ifndef CTD_H
#define CTD_H

#include <cdpr_controllers/qp.h>
#include <cdpr/cdpr.h>

// this class implements all candidates for CTD's
// this way we do not have the same code in all sources


class CTD
{
public:

    // how we perform the CTD
    typedef enum
    {
        minA, minW, minT, noMin, minAA, minTs, closed_form
    } minType;


    CTD(CDPR &robot, minType _control, bool warm_start = true);

    // will look for a solution in [tau +- dTau_max]
    void ForceContinuity(double _dTau_max) {dTau_max = _dTau_max;}

    vpColVector ComputeDistribution(vpMatrix &W, vpColVector &w);

    // for minA
    void GetAlpha(double &a)
    {
        if(control == minA || control == minAA )
            a = x[n];
        else
            a = 0;
    }


protected:
    minType control;
    int n;
    double tauMin, tauMax;

    vpMatrix Q, A, C, W;
    vpColVector r, b, d, x;
    vpSubColVector tau;

    bool update_d;
    double dTau_max;

    bool reset_active;
    std::vector<bool> active;


};

#endif // CTD_H
