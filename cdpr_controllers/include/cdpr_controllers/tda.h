#ifndef TDA_H
#define TDA_H

#include <cdpr_controllers/qp.h>
#include <cdpr/cdpr.h>
#include <cmath>
#include <std_msgs/Float32MultiArray.h>
#include <ros/publisher.h>

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


    TDA(CDPR &robot, ros::NodeHandle &_nh, minType _control, bool warm_start = false);

    // will look for a solution in [tau +- dTau_max]
    void ForceContinuity(double _dTau_max) {dTau_max = _dTau_max;}

    vpColVector ComputeDistribution(vpMatrix &W, vpColVector &w);

    // for minA
    void GetAlpha(double &a)
    {
        if(control == minA )
            a = x[n];
    }
    // for Barycentric algorithm
    void GetVertices(double &a)
    {
        if(control == Barycenter )
            a = vertices.size();
    }


protected:
    minType control;
    int n;
    double tauMin, tauMax;

    vpMatrix Q, A, C, W;
    vpColVector r, b, d, x;
    vpSubColVector tau, alpha;

    bool update_d;
    double dTau_max, dAlpha;

    bool reset_active;
    std::vector<bool> active;
    std::vector<vpColVector> vertices;


     // declaration of closed form
     vpColVector f_m, f_v, w_, tau_;
     vpMatrix W_;

    // declaration of Barycenter
    double m;
    vpColVector  lambda, F, p, v_1, v_2, v_c;
    vpMatrix kerW, H, ker;
    // publisher to barycenter plot
    ros::Publisher bary_pub;

};

#endif // TDA_H
