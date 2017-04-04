#include <cdpr_controllers/ctd.h>

using std::cout;
using std::endl;

CTD::CTD(CDPR &robot, minType _control, bool warm_start)
{
    // number of cables
    n = robot.n_cables();

    // forces min / max
    robot.tensionMinMax(tauMin, tauMax);

    control = _control;
    dTau_max = 1;
    update_d = false;

    x.resize(n);

    reset_active = !warm_start;
    active.clear();

    // prepare variables
    if(control == minT)
    {
        // min |tau|
        //  st W.tau = w        // assumes the given wrench is feasible
        //  st t- < tau < tau+

        // min tau
        Q.eye(n);
        r.resize(n);
        // equality constraint
        A.resize(6,n);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if (control == minTs)
    {
        Q.resize(n,n);
        for(unsigned int i=0;i<n;++i)
            for(unsigned int k=0;k<n;++k)
                    Q[i][k]=1;
        r.resize(n);
        // equality constraint
        A.resize(6,n);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minW)
    {
        // min |W.tau - w|      // does not assume the given wrench is feasible
        //   st t- < tau < t+

        Q.resize(6,n);
        r.resize(6);
        // no equality constraints
        A.resize(0,n);
        b.resize(0);
        // min/max tension constraints
        C.resize(2*n,n);
        d.resize(2*n);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
    }
    else if(control == minA)
    {
        // min |tau| - alpha
        //  st W.tau = alpha.w
        //  st 0 < alpha < 1
        //  st t- < tau < t+
        x.resize(n+2);  // x = (tau, alpha)

        Q.eye(n+2);Q *= 1./tauMax;
        r.resize(n+2);r[n] = Q[n][n] = r[n+1] = Q[n+1][n+1] = n*tauMax;
        // equality constraints
        A.resize(6,n+2);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*n+4,n+2);
        d.resize(2*n+4);
        for(int i=0;i<n;++i)
        {
            C[i][i] = 1;
            d[i] = tauMax;
            C[i+n][i] = -1;
            d[i+n] = -tauMin;
        }
        // constraints on alpha
        d[2*n] = d[2*n+2] = 1;
        C[2*n][n] = C[2*n+2][n+2] = 1;
        C[2*n+1][n+1] = C[2*n+3][n+3] = -1;
    }
    else if (control == minAA)
    {
        x.resize(n+1); // x = (tau, alpha)

        Q.eye(n+1); Q *= 1./tauMax;
        r.resize(n+1);
        Q[n][n]=r[n]=n*tauMax;
        // equality constraints
        A.resize(6,n+1);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*(n+1), (n+1));
        d.resize(2*(n+1));
        for(unsigned int i=0;i<n;++i)
            {
                     // f < fmax
                    C[i][i] = 1;
                    // -f < -fmin
                    C[i+n][i] = -1;
                    d[i] = tauMax;
                    d[i+n] = -tauMin;
             }
                    C[2*n][n]=1;
                    C[2*n+1][n]=-1;
                    d[2*n] = 1;
                    d[2*n+1]= 0;
    }
    tau.init(x, 0, n);
}



vpColVector CTD::ComputeDistribution(vpMatrix &W, vpColVector &w)
{
    if(reset_active)
        for(int i=0;i<active.size();++i)
            active[i] = false;

    if(update_d && control != noMin)
    {
        for(unsigned int i=0;i<n;++i)
        {
            d[i] = std::min(tauMax, tau[i]+dTau_max);
            d[i+n] = -std::max(tauMin, tau[i]-dTau_max);
        }
    }

    if(control == noMin)
        x = W.pseudoInverse() * w;
    else if(control == minT)
        solve_qp::solveQP(Q, r, W, w, C, d, x, active);
        else if(control == minTs)
        solve_qp::solveQP(Q, r, W, w, C, d, x, active);
    else if(control == minW)
        solve_qp::solveQPi(W, w, C, d, x, active);
    else if(control== minAA)  // control = minAA
    {
        A.insert(W,0,0);
        for(int i=0;i<6;++i)
        {
                A[i][n]= - w[i];
        }
        solve_qp::solveQP(Q, r, A, b, C, d, x, active);
    }
    else    // control = minA
    {
        for(int i=0;i<6;++i)
        {
            for(int j=0;j<n;++j)
                A[i][j] = W[i][j];
            if(i < 3)
                A[i][n] = -w[i];
            else
                A[i][n+1] = -w[i];
        }
        solve_qp::solveQP(Q, r, A, b, C, d, x, active);
  //      cout << "alpha = " << alpha[0] << endl;
  //      cout << "checking W.tau - a.w: " << (W*tau - alpha[0]*w).t() << endl;
    }
 //   cout << "Residual: " << (W*tau - w).t() << fixed << endl;

    cout << "check constraints :" << endl;
    for(int i=0;i<n;++i)
            cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;

    update_d = dTau_max;

    return tau;
}

