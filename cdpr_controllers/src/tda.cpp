#include <cdpr_controllers/tda.h>

using std::cout;
using std::endl;
using std::vector;

TDA::TDA(CDPR &robot, ros::NodeHandle &_nh, minType _control, bool warm_start)
{
    // number of cables
    n = robot.n_cables();

    // mass of platform
    m=robot.mass();

    // forces min / max
    robot.tensionMinMax(tauMin, tauMax);

    control = _control;

    dAlpha= 0.001;
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
    else if (control == minA)
    {
        // min |tau| - alpha
        //  st W.tau = alpha.w
        //  st 0 < alpha < 1
        //  st t- < tau < t+
        x.resize(n+1); // x = (tau, alpha)
        Q.eye(n+1); Q *= 1./tauMax;
        r.resize(n+1);
        Q[n][n]=r[n]=7500;
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
    else if ( control == closed_form)
    {
        // tau=f_m+f_v
        //  f_m=(tauMax+tauMin)/2
        //  f=f_m- (W^+)(w+W*f_m)
        // min ||f||2

        f_m.resize(n);
        f_v.resize(n);
        w_.resize(6);
        W_.resize(6,n);
        tau_.resize(n);
        // no equality constraints
        d.resize(2*n);
        for (unsigned int i = 0; i <n; ++i)
        {
            f_m[i]=(tauMax+tauMin)/2;
            d[i] =tauMax;
            d[i+n] = - tauMin;
        }
    }
    else if ( control == Barycenter)
    {
        // publisher to plot
        bary_pub = _nh.advertise<std_msgs::Float32MultiArray>("barycenter", 1);

        H.resize(n , n-6);
        d.resize(2*n);
        lamda.resize(2);
        F.resize(2);
        // particular solution from the pseudo Inverse
        p.resize(n);
        ker.resize(n-6,n-6);
        // the vertex of the polygon
        v_1.resize(n-6);
        v_2.resize(n-6);
        v_c.resize(n-6);
        for (unsigned int i = 0; i <n; ++i)
        {
            d[i] =tauMax;
            d[i+n] = - tauMin;
        }
    }
    tau.init(x, 0, n);
    //alpha.init(x, n, 1);
}



vpColVector TDA::ComputeDistribution(vpMatrix &W, vpColVector &w)
{
    if(reset_active)
        for(int i=0;i<active.size();++i)
            active[i] = false;

    if(update_d && control != noMin && control != closed_form)
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
    else if(control == minW)
        solve_qp::solveQPi(W, w, C, d, x, active);
    else if(control== minA)  // control = minA
    {
        A.insert(W,0,0);
        for(int i=0;i<6;++i)
            A[i][n]= - w[i];
        solve_qp::solveQP(Q, r, A, b, C, d, x, active);
        //      cout << "alpha = " << alpha[0] << endl;
        //      cout << "checking W.tau - a.w: " << (W*tau - alpha[0]*w).t() << endl;
    }

    else if( control == closed_form)
    {
        x= f_m + W.pseudoInverse() * (w - (W*f_m));
        f_v= x- f_m;
        //compute the range limit of f_v
        norm_2 = sqrt(f_v.sumSquare());
        w_=w; W_=W ; num_r= n-6;
        //cout << "redundancy " << num_r<< endl;
        range_lim= sqrt(m)*(tauMax+tauMin)/4;
        //cout << "the maximal limit" << range_lim<<endl;
        if ( norm_2 <= range_lim )
        {
            for (int i = 0; i < n; ++i)
            {
                if ( x[i] > (tauMax+0.001) && num_r >=0)
                {
                    cout << "previous tensions" << "  "<< i<<x.t()<<endl;
                    cout << " i"<<"  "<< endl;
                    // re- calculate the external wrench with maximal element
                    w_= -tauMax*W_.getCol(i)+w_;
                    tau_[i]=tauMax;
                    f_m[i]=0;
                    // drop relative column
                    W_[0][i]=W_[1][i]=W_[2][i]=W_[3][i]=W_[4][i]=W_[5][i]=0;
                    //compute the tensions again without unsatisfied component
                    x = f_m + W_.pseudoInverse()*(w_- (W_*f_m));
                    // reduce the redundancy order
                    num_r--;
                    // construct the latest TD with particular components which equal to minimum and maximum
                    x=tau_+x;
                    // initialize the index in order to inspect from the first electment
                    i=0;
                    cout << "larger tensions" << "  "<<x.t()<<endl;
                }
                else if (x[i] < (tauMin-0.001) && num_r >=0)
                {
                    cout << "previous tensions" << "  "<<x.t()<<endl;
                    cout << " i"<<"  "<< i<<endl;
                    // re- calculate the external wrench with minimal element
                    w_= -tauMin*W_.getCol(i)+w_;
                    tau_[i]=tauMin;
                    f_m[i]=0;
                    // drop relative column
                    W_[0][i]=W_[1][i]=W_[2][i]=W_[3][i]=W_[4][i]=W_[5][i]=0;
                    //compute the tensions again without unsatisfied component
                    x = f_m + W_.pseudoInverse()*(w_- (W_*f_m));
                    // reduce the redundancy order
                    num_r--;
                    // construct the latest TD with particular components which equal to minimum and maximum
                    x=tau_+x;
                    // initialize the index in order to inspect from the first electment
                    i=0;
                    cout << "small tensions" << "  "<<x.t()<<endl;
                }
                else if (num_r < - 0.09)
                    cout << "no solution exists" << endl;
            }
        }
        else
            cout << "no feasible tension distribution" << endl;
        cout << "The closed form is implemented"<< endl;
    }

    else if ( control == Barycenter)
    {
        // compute the kernel of matrix W
        W.kernel(kerW);
        // obtain the particular solution of tensions
        p=W.pseudoInverse() * w;
        // lower bound
        vpColVector A = -p + tauMax;
        // upper bound
        vpColVector B = -p + tauMin;

        // construct the multiple kernel matrix
        H = kerW.t();

        // build and publish H A B
        std_msgs::Float32MultiArray msg;
        msg.data.resize(32);
        for(int i=0;i<8;++i)
        {
            msg.data[4*i] = H[i][0];
            msg.data[4*i+1] = H[i][1];
            msg.data[4*i+2] = A[i];
            msg.data[4*i+3] = B[i];
        }
        bary_pub.publish(msg);

        // we look for points such as A <= H.x <= B

        // initialization of parameters
        v_c[0]=0.0 ; v_c[1]=0.0;
        vertices.clear();
        // cout << "vertices:" <<"  "<<vertices <<endl;
        // construct the 2x2 subsystem of linear equations in order to gain the intersection points in preimage
        for (int i = 0; i < n; ++i)
        {
            ker[0][0]=H[i][0];
            ker[0][1]=H[i][1];
            for(int j=0;j<n;++j)
            {
                if(i != j)
                {
                    ker[1][0]=H[j][0];
                    ker[1][1]=H[j][1];
                    // pre-compute the inverse
                    ker = ker.inverseByQR();

                    for(double u: {A[i],B[i]})
                    {
                        for(double v: {A[j],B[j]})
                        {
                            // solve this intersection
                            F[0] = u;F[1] = v;
                            lamda = ker * F;

                            // check constraints
                            if((H*lamda - A).getMinValue() >= 0 && (H*lamda - A).getMaxValue() <= 0)
                                vertices.push_back(lamda);
                        }
                    }
                }
            }
        }

        // print the  satisfied vertices  number
        cout << "number of vertex:" << "  "<< vertices.size() << endl;

        vpColVector centroid(2);


        if(vertices.size())
        {

            // compute centroid
            for(auto &vert: vertices)
                centroid += vert;
            centroid /= vertices.size();

            // compute actual CoG if more than 1 point
            if(vertices.size() > 1)
            {

                // re-order according to angle to centroid
                std::sort(vertices.begin(),vertices.end(),[&centroid](vpColVector &v1, vpColVector &v2)
                    {return atan2(v1[1]-centroid[1],v1[0]-centroid[0]) > atan2(v2[1]-centroid[1],v2[0]-centroid[0]);}); // may be the opposite

                // compute CoG
                vertices.push_back(vertices[0]);
                double a=0,v;
                centroid = 0;
                for(int i=1;i<vertices.size();++i)
                {
                    v = vertices[i-1][0]*vertices[i][1] - vertices[i][0]*vertices[i-1][1];
                    a += v;
                    centroid[0] += v*(vertices[i-1][0] + vertices[i][0]);
                    centroid[1] += v*(vertices[i-1][1] + vertices[i][1]);
                }
                centroid /= 3*a;
            }
            cout << "the barycenter" << "  "<< centroid.t() << endl;
            x = p+ H*centroid;

            cout << "check constraints :" << endl;
            for(int i=0;i<n;++i)
                cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
        }
        else
        {
            cout << "No appropriate TDA " << endl;
        }
    }
    update_d = dTau_max;
    return tau;


}

