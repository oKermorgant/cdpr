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
        minA, minW, minT, noMin,  closed_form, Barycenter, minG, cgal
    } minType;

   
    TDA(CDPR &robot, ros::NodeHandle &_nh, minType _control, bool warm_start = false);

    // will look for a solution in [tau +- dTau_max]
    void ForceContinuity(double _dTau_max) {dTau_max = _dTau_max;}
     void Weighing(double lambda){ _lambda = lambda;}

    vpColVector ComputeDistribution(vpMatrix &W, vpColVector &w);
    vpColVector ComputeDistributionG(vpMatrix &W, vpColVector &ve, vpColVector &pe, vpColVector &w );

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
            a =num_v;
    }
    void GetGains(vpColVector &a)
    {
        if(control == minG)
            a [0] = x[8]; 
            a[1] = x[9];
    }

protected:
    minType control;
    int n,index, num_v;
    double tauMin, tauMax;

    vpMatrix Q, A, C;
    vpColVector r, b, d, x, wp;
    vpSubColVector tau;

    bool update_d;
    double dTau_max, dAlpha,  _lambda;

    bool reset_active;
    std::vector<bool> active;
    std::vector<vpColVector> vertices;



     // declaration of closed form
     vpColVector f_m, f_v, w_;
     

    // declaration of Barycenter
    double m;
    vpColVector  lambda, F, p;
    vpMatrix kerW, H, ker, ker_inv;
    // publisher to barycenter plot
    ros::Publisher bary_pub;

};

#endif // TDA_H
