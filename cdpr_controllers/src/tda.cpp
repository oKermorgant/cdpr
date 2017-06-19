#include <cdpr_controllers/tda.h>
#include <visp/vpIoTools.h>

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

    // indicator of number of interaton which has infeasible  tension
    index=0;

    control = _control;
    update_d = false;

    x.resize(n);

    reset_active = !warm_start;
    cout << "reset_active" << reset_active << endl;
    active.clear();

    // prepare variables
    if(control == minT)
    {
        // min |tau|
        //  st W.tau = w        // assume the given wrench is feasible dependent on the gains
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
        // min |tau| - |lambda.(1-alpha)|
        //  st W.tau = alpha.w+(1-alpha).wp
        //  st 0 < alpha < 1
        //  st t- < tau < t+

        x.resize(n+1); // x = (tau, alpha)
        Q.eye(n+1); Q *= 1./tauMax;
        r.resize(n+1);
        wp.resize(6);
        Q[n][n]=r[n]=7000;
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

        f_m.resize(n);
        f_v.resize(n);
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
        // initialize the matrix
        std::vector<double>::size_type num_v;
        H.resize(n , n-6);
        d.resize(2*n);
        lambda.resize(2);
        F.resize(2);
        // particular solution from the pseudo Inverse
        p.resize(n);
        ker.resize(n-6,n-6);
        ker_inv.resize(n-6,n-6);
        for (unsigned int i = 0; i <n; ++i)
        {
            d[i] =tauMax;
            d[i+n] = - tauMin;
        }
    }
    else if ( control == minG)
    {
        // min |tau| +|lambda.(Kp-alpha1)|+|lambda.(Kd-alpha2)|
        //  st W.tau =Ip(xdd+Kd.v_e+Kp.p_e)-wg
        //  st 1< Kp < 100
        //  st 1 <kd <100
        //  st t- < tau < t+

        // publisher to plot
        bary_pub = _nh.advertise<std_msgs::Float32MultiArray>("barycenter", 1);
        H.resize(n , n-6);
        // particular solution from the pseudo Inverse
        p.resize(n);
        ker.resize(n-6,n-6);
        ker_inv.resize(n-6,n-6);

        x.resize(n+2);  // x = (tau, Kp, Kd)
        r.resize(n+2);
        Q.eye(n+2);  Q *= 1./tauMax;
        // gains for position
        Q[n][n]=100; r[n] =16*100;
        Q[n+1][n+1] = 100; r[n+1]=8*100;

/*         // gains for position
        Q[n][n]=1; r[n] = 1;
        Q[n+1][n+1] = 1; r[n+1]=1;
        // gains for orientation
        Q[n+2][n+2]=1; r[n+2] =1;
        Q[n+3][n+3] = 1; r[n+3]= 1;*/

        // equality constraints
        A.resize(6,n+2);
        b.resize(6);
        // min/max tension constraints
        C.resize(2*(n+2), (n+2));
        d.resize(2*(n+2));
        for(unsigned int i=0;i<n;++i)
        {
            // f < fmax
            C[i][i] = 1;      
            // -f < -fmin
            C[i+n][i] = -1;
            d[i] = tauMax;
            d[i+n] = -tauMin;
        }
        C[2*n][n]=C[2*n+1][n+1]=1;//C[2*n+2][n+2]=C[2*n+3][n+3]=1;
        C[2*n+2][n]=C[2*n+3][n+1]=-1;//C[2*n+6][n+2]=C[2*n+7][n+3]= -1;
        d[2*n] = d[2*n+1]=100.0;
        d[2*n+2]= d[2*n+3]= -0.001;//d[2*n+6]= d[2*n+7]=- 1.0;      
    }
    else if(control == cgal)
    {   
           // min tau
        Q.eye(n);
        r.resize(n);
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
    tau.init(x, 0, n);
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
        cout << "Using multiplier" << endl;
        A.insert(W,0,0);
        for(int i=0;i<6;++i)
            A[i][n]= - w[i]+wp[i];
        b=wp;
        solve_qp::solveQP(Q, r, A, b, C, d, x, active);
        wp=x[n]*w+(1-x[n])*wp;
         if ( (C*x-d).getMaxValue() < 0.01)
             cout << " satisfy the inequality constraint" << endl;
        else
              index++;
    }

    else if( control == closed_form)
    {   
        // declaration 
        cout << "Using closed form" << endl;
        int num_r, index=0;
        vpColVector fm(n), tau_(n), w_(6);
        vpMatrix W_(6,n);
        double range_lim, norm_2;
        x= f_m + W.pseudoInverse() * (w - (W*f_m));
        f_v= x- f_m;
        fm=f_m;
        //compute the range limit of f_v
        norm_2 = sqrt(f_v.sumSquare());
        w_=w; W_=W ; num_r= n-6;
        //cout << "redundancy " << num_r<< endl;
        range_lim= sqrt(m)*(tauMax+tauMin)/4;
        //cout << "the maximal limit" << range_lim<<endl;
       for (int i = 0; i < n; ++i)
        {
            if ( norm_2 <= range_lim )
            {                 
                    if (  (x.getMaxValue() > tauMax || x.getMinValue() < tauMin ) && num_r >0  )
                    {
                        // search the relevant element which is unsatisfied
                        for (int j = 0; j < n; ++j)
                          {  
                            if(  x[j] == x.getMinValue() && x.getMinValue() < tauMin )
                                    i = j;
                            else if  (x[j] == x.getMaxValue() && x.getMaxValue() > tauMax)
                                    i=j;
                         }
                        cout << "previous tensions" << "  "<<x.t()<<endl;
                        cout << " i"<<"  "<< i <<endl;
                        // reduce the redundancy order
                        num_r--;
                        cout << "number of redundancy"<<"  "<< num_r<<endl;
                        // re- calculate the external wrench with maximal element
                        if ( x.getMaxValue() > tauMax)
                         {
                            w_= -tauMax*W_.getCol(i)+w_;
                            tau_[i]=tauMax;
                        }
                        else 
                        {
                            w_= - tauMin * W_.getCol(i)+w_;
                            tau_[i]=tauMin;
                        }
                        cout << "torque" << tau_.t() << endl;

                        fm[i]=0;

                        // drop relative column
                        W_[0][i]=W_[1][i]=W_[2][i]=W_[3][i]=W_[4][i]=W_[5][i]=0;

                        //compute the tensions again without unsatisfied component
                        x = fm + W_.pseudoInverse()*(w_- (W_*fm));

                        // construct the latest TD with particular components which equal to minimum and maximum
                        x=tau_+x;
                        cout << "tensions" << x.t() << endl;

                        // compute the force limit
                        f_v = x - f_m;
                        norm_2 = sqrt(f_v.sumSquare());
                        // initialize the index in order to inspect from the first electment
                        i = 0;
                    }
                    else if(num_r <0)
                        cout << "no feasible redundancy existing" << endl;
              }
        else
            cout << "no feasible tension distribution" << endl;
        }
    }

    else if ( control == Barycenter)
    {
        cout << "Using Barycenter" << endl;
        int inter=0;
        // compute the kernel of matrix W
        W.kernel(kerW);
        // obtain the particular solution of tensions
        p=W.pseudoInverse() * w;
        // lower and upper bound
        vpColVector A = -p, B = -p;
        for(int i=0;i<8;++i)
        {
            A[i] += tauMin;
            B[i] += tauMax;
        }

        // construct the projection matrix
        H = kerW.t();

        // build and publish H A B
        std_msgs::Float32MultiArray msg;
        msg.data.resize(32);
        for(int i=0; i<8 ;++i)
        {
            msg.data[4*i] = H[i][0];
            msg.data[4*i+1] = H[i][1];
            msg.data[4*i+2] = A[i];
            msg.data[4*i+3] = B[i];
        }
        bary_pub.publish(msg);

        // we look for points such as A <= H.x <= B

        // initialize vertices vector
        vertices.clear();

        // construct the 2x2 subsystem of linear equations in order to gain the intersection points in preimage
        for (int i = 0; i < n; ++i)
        {
            ker[0][0]=H[i][0];
            ker[0][1]=H[i][1];
            // eliminate the same combinations with initial value (i+1)
            for(int j=(i+1); j<n; ++j)
            {
                    ker[1][0]=H[j][0];
                    ker[1][1]=H[j][1];
                    // pre-compute the inverse
                    ker_inv = ker.inverseByLU();
                    // the loop from A[i] to B[i];
                    for(double u: {A[i],B[i]})
                    {
                        for(double v: {A[j],B[j]})
                        {
                            // solve this intersection
                            F[0] = u;F[1] = v;
                            lambda = ker_inv * F;
                            inter++;
                            // check constraints, must take into account the certain threshold
                            if((H*lambda - A).getMinValue() >= - 1e-6 && (H*lambda - B).getMaxValue() <= 1e-6)
                                 vertices.push_back(lambda);
                        }
                    }
            }
        }
        cout << "the total amount of intersectioni points:" <<"  "<<inter << endl;
        // print the  satisfied vertices  number
        num_v = vertices.size();
        cout << "number of vertex:" << "  "<<num_v<< endl;
        for (int i = 0; i < vertices.size(); ++i)
           cout << "vertex " << "  "<< vertices[i].t()<<endl;


        vpColVector centroid(2);
        vpColVector ver(2), CoG(2);

        if( vertices.size() )
        {
            // compute centroid
            for(auto &vert: vertices)
                centroid += vert;
            centroid /= vertices.size();

            // compute actual CoG if more than 2 points
            if(vertices.size() > 2)
            {
                // re-order according to angle to centroid in clockwise order
                std::sort(vertices.begin(),vertices.end(),[&centroid](vpColVector v1, vpColVector v2)
                    {return atan2(v1[1]-centroid[1],v1[0]-centroid[0]) > atan2(v2[1]-centroid[1],v2[0]-centroid[0]);}); 

/*                // compute CoG with trianglation alogorithm
                double a=0,v;
                centroid = 0; CoG=0; 
                for (int j= 1; j < (vertices.size()-1) ; ++j)
                {
                    ver+=(vertices[0]+vertices[j]+vertices[j+1]);
                    ver/=3;
                    v=(vertices[0][0]*vertices[j][1]+vertices[j][0]*vertices[j+1][1]+vertices[j+1][0]*vertices[0][1]
                        -vertices[0][0]*vertices[j+1][1]-vertices[j][0]*vertices[0][1]-vertices[j+1][0]*vertices[j][1])/2;
                    CoG+=ver*v;
                    a+=v;
                }
                centroid= CoG/a;*/

           // compute CoG directly from convex polygon
                vertices.push_back(vertices[0]);
                double a=0,v;
                centroid = 0;
                for(int i=1;i< vertices.size();++i)
                {
                    v = vertices[i-1][0]*vertices[i][1] - vertices[i][0]*vertices[i-1][1];
                    a += v;
                    centroid[0] += v*(vertices[i-1][0] + vertices[i][0]);
                    centroid[1] += v*(vertices[i-1][1] + vertices[i][1]);
                }
                centroid /= 3*a;
            }
            x = p+ H*centroid;
            cout << "the barycenter" << "  "<< centroid.t() << endl;
        }
        else 
            cout << "there is no vertex existing"<< endl;
    }
/*    else if (control == cgal)
        {   
            cout<< "using cgal"<< endl;
             solve_qp::solveQP(Q, r, W, w, C, d, x, active);
             // initialize the Q matrix
            double H1[] = {1,0,0,0,0,0,0,0};
            double H2[] = {0,1,0,0,0,0,0,0};    
            double H3[] = {0,0,1,0,0,0,0,0} ;    
            double H4[] = {0,0,0,1,0,0,0,0};
            double H5[] = {0,0,0,0,1,0,0,0} ;    
            double H6[] = {0,0,0,0,0,1,0,0} ; 
            double H7[] = {0,0,0,0,0,0,1,0};
            double H8[] = {0,0,0,0,0,0,0,1} ;    
            double*  H[] = {H1, H2, H3, H4, H5, H6, H7, H8}; 

            double A1[] = {W[0][0], W[1][0],W[2][0],W[3][0],W[4][0],W[5][0]};
            double A2[] = {W[0][1], W[1][1],W[2][1],W[3][1],W[4][1],W[5][1]};
            double A3[] = {W[0][2], W[1][2],W[2][2],W[3][2],W[4][2],W[5][2]};
            double A4[] = {W[0][3], W[1][3],W[2][3],W[3][3],W[4][3],W[5][3]};
            double A5[] = {W[0][4], W[1][4],W[2][4],W[3][4],W[4][4],W[5][4]};
            double A6[] = {W[0][5], W[1][5],W[2][5],W[3][5],W[4][5],W[5][5]};
            double A7[] = {W[0][6], W[1][6],W[2][6],W[3][6],W[4][6],W[5][6]};
            double A8[] = {W[0][7], W[1][7],W[2][7],W[3][7],W[4][7],W[5][7]};

            double*  A[] = {A1, A2, A3, A4, A5, A6, A7, A8}; 
            double   b[] = {w[0], w[1],w[2],w[3],w[4],w[5]}; 

            bool fl[] = {true, true, true, true, true, true, true, true};
            bool fu[] = {true, true, true, true, true, true, true, true}; 

            CGAL::Const_oneset_iterator<CGAL::Comparison_result>  r(CGAL::EQUAL); 

            double lb[] = {tauMin,tauMin,tauMin,tauMin,tauMin,tauMin,tauMin,tauMin};
            double ub[] = {tauMax,tauMax,tauMax,tauMax,tauMax,tauMax,tauMax,tauMax};

            double c[] = {0,0,0,0,0,0,0,0};
            double  c0   = 0;  

            // now construct the quadratic program; the first two parameters are
             // the number of variables and the number of constraints (rows of A)
             Program qp (8, 6, A, b, r, fl, lb, fu, ub, H, c, c0);
             Solution SOL = CGAL::solve_quadratic_program(qp, ET());

            Solution::Variable_value_iterator it = SOL.variable_values_begin();
            Solution::Variable_value_iterator end = SOL.variable_values_end();

             int index=0;
             for (; it != end; ++it) 
            {    
                 x[index] = *it.to_double();
                 index++;
             }

            //cout<< "the solution"<< x.t()<<endl;
            cout<< "the return solution:\n"<<"  "<<SOL<<endl;
        }*/
    else
        cout << "No appropriate TDA " << endl;
   cout << "check constraints :" << endl;
            for(int i=0;i<n;++i)
                cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
    update_d = dTau_max;
    return tau;
}

vpColVector TDA::ComputeDistributionG(vpMatrix &W, vpColVector &ve, vpColVector &pe, vpColVector &w )
{   
        int rank_A; vpMatrix kernelA;
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

        A.insert(W,0,0);
        for(int i=0;i<6;++i)
        {
            // position
            A[i][n] = - pe[i];
            A[i][n+1] = -ve[i]; 
/*            // oritentaion
            A[i+3][n+2] = -pe[i+3];
            A[i+3][n+3] = -ve[i+3];*/
        }
        //cout<<"the matrix A:"<< "  "<<A<< endl;
        //cout<<"the matrix w:"<< "  "<<w.t()<< endl;
        //rank_A=A.kernel(kernelA);
        b = w;
        solve_qp::solveQP(Q, r, A, b, C, d, x, active);
        cout<<"the difference:"<< "  "<<(A*x-b).t()<< endl;

        vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/Q_matrix", Q);
        vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/A_matrix", A);
        vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/r_matrix", r);
        vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/b_matrix", b);
        vpMatrix::saveMatrixYAML("/home/" + vpIoTools::getUserName() + "/Results/matrices/d_matrix", d);
        //cout<<"the rank of A:"<< "  "<<rank_A<< endl;
        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         // compute the kernel of matrix W
        W.kernel(kerW);
        // obtain the particular solution of tensions
        p=W.pseudoInverse() * w;
        // lower and upper bound
        vpColVector A = -p, B = -p;
        for(int i=0;i<8;++i)
        {
            A[i] += tauMin;
            B[i] += tauMax;
        }
        // construct the projection matrix
        H = kerW.t();
        // build and publish H A B
        std_msgs::Float32MultiArray msg;
        msg.data.resize(32);
        for(int i=0; i<8 ;++i)
        {
            msg.data[4*i] = H[i][0];
            msg.data[4*i+1] = H[i][1];
            msg.data[4*i+2] = A[i];
            msg.data[4*i+3] = B[i];
        }
        bary_pub.publish(msg);
        //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        //     cout << "checking W.tau - a.w: " << (W*tau - alpha[0]*w).t() << endl;
        cout << "[Kpp, Kdp]:" << "  "<<"["<<x[8]<< "  "<< x[9]<<"]"<< endl;
        cout << "check constraints :" << endl;
            for(int i=0;i<n;++i)
                cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
    update_d = dTau_max;
    return tau;
}
