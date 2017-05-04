#ifndef TDA_H
#define TDA_H

#include <cdpr_controllers/qp.h>
#include <cdpr/cdpr.h>
#include <cmath>

// this class implements all candidates for TDA's
// this way we do not have the same code in all sources


class TDA
{
public:

    // how we perform the TDA
    typedef enum
    {
        minA, minW, minT, noMin,  closed_form, Barycenter
    } minType;


    TDA(CDPR &robot, minType _control, bool warm_start = false);

    // will look for a solution in [tau +- dTau_max]
    void ForceContinuity(double _dTau_max) {dTau_max = _dTau_max;}

    vpColVector ComputeDistribution(vpMatrix &W, vpColVector &w);

    // for minA
    void GetAlpha(double &a)
    {
        if(control == minA )
            a = x[n];
        else
            a = 0;
    }


protected:
    minType control;
    int n, num, inter_n, rank, num_v;
    double tauMin, tauMax, m, area;

    vpMatrix Q, A, C, W, kerW, H, ker;
    vpColVector r, b, d, x, f_m, f_v, lamda, F, p, sol, v_1, v_2, v_c;
    vpSubColVector tau, alpha;

    bool update_d;
    double dTau_max, dAlpha;

    bool reset_active;
    std::vector<bool> active;
    std::vector<vpColVector> vertices;


};

#endif // TDA_H
