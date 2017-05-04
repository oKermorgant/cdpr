#include <cdpr_controllers/tda.h>

using std::cout;
using std::endl;
using std::vector;

TDA::TDA(CDPR &robot, minType _control, bool warm_start)
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
        //  W.tau-w=0
        // min ||f_v||2
        //  st 1/2*f_m < f_v < 1/2* sqrt(m)*f_m

        f_m.resize(n);
        f_v.resize(n);
        // min f_v
        Q.eye(n);
        r.resize(n);
        // no equality constraints
        d.resize(2*n);
        C.resize(2*n,n);
        // equality constraints
        A.eye(n);
        b.resize(n);
        for (unsigned int i = 0; i <n; ++i)
        {
            f_m[i]=(tauMax+tauMin)/2;
            C[i][i] = 1;
            d[i] =tauMax; //(tauMax-tauMin)/2;
            C[i+n][i] = -1;
            d[i+n] = - tauMin; //(tauMax-tauMin)/2;
        }    
        r=f_m;
    }
    else if ( control == Barycenter)
    {
        H.resize(2*n , n-6);
        sol.resize(2*n);
        d.resize(2*n);
        lamda.resize(2);
        F.resize(2);
        p.resize(n);
        ker.resize(n-6,n-6);
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
         tau= f_m + W.pseudoInverse()*(w - (W*f_m)); 
         //solve_qp::solveQP(Q, r, A, b, C, d, x, active);
         //solve_qp::solveQP(Q, r, A, b, C, d1, f_v, active);
         //tau=f_m+f_v;
         cout << "The closed form is implemented"<< endl;
    }
   
   else if ( control == Barycenter)
    {
        // compute the kernel of matrix W
        rank=W.kernel(kerW);
        cout << "rank of structure matrix:"<< "  "<<rank<<endl;
        // obtain the particular solution of tensions
        p=W.pseudoInverse() * w;
        // construct the kernel matrix
        H.insert(kerW.t(),0,0);
        H.insert(kerW.t(),n,0);
        // initialization of parameters
        inter_n=0;
        area=0.0; 
        v_c[0]=0.0 ; v_c[1]=0.0;
        vertices.clear();

        //cout << "vertices:" <<"  "<<vertices <<endl;
        // construct the 2x2 subsystem of linear equations in oder to gain the intersection points in preimage
        for (int i = 0; i < 2*n; ++i)
        {      
               ker[0][0]=H[i][0];
               ker[0][1]=H[i][1];
               if (i<n)
                    F[0]=tauMax-p[i];
               else
                    F[0]=tauMin-p[i-n];
               for (int k = (i+1); k <2*n ; ++k)
               {     
                    num=0;
                    if ( (k-i) != n)
                    {
                        ker[1][0]=H[k][0];
                        ker[1][1]=H[k][1];    
                       if (k<n)
                            F[1]=tauMax-p[k];
                        else
                            F[1]=tauMin-p[k-n]; 
                        inter_n++;                  
                    }
                    lamda= ker.inverseByLU()*F;
                    //cout << "lamda in the r dimensional space:" <<"  " << lamda.t()<< endl;
                    sol=kerW.t()*lamda;
                    // check whether the intersection point satisfies all inequality equations
                    for (int j = 0; j < n; ++j)
                        if ( sol[j] <= (tauMax-p[j]) && sol[j] >=  (tauMin-p[j]) )
                            num=num+1;
                    if ( num == n)
                         vertices.push_back(lamda);
                 }
            }
            
            num_v = vertices.size();
            // print the  satisfied vertices  number
            cout << "number of vertex:" << "  "<< num_v << endl;
            cout << "the number of intersection points:"<< "  "<< inter_n << endl;

            // compute the barycenter v_c
            if (num_v >= 3)
            {
                for (int i = 0; i < (num_v-1); ++i)
                {
                    v_1=vertices[i]; v_2=vertices[i+1];
                    area+= v_1[0]*v_2[1] - v_1[1]*v_2[0];
                    v_c[0]+=(v_1[0]+v_2[0])*(v_1[0]*v_2[1] - v_2[0]*v_1[1]);
                    v_c[1]+=(v_1[1]+v_2[1])*(v_1[0]*v_2[1] - v_2[0]*v_1[1]); 
                }
                area = area/2;
                v_c = v_c / (6*area);
            }
            else if (num_v ==2 || num_v ==1)
              {
                  for (int i = 0; i < num_v; ++i)
                        v_c +=vertices[i];
                   v_c = v_c / num_v;
              }
            else
                cout << "no proper lamda"<< endl;

            cout << "the barycenter" << "  "<< v_c.t() << endl;
            x = p+kerW.t()*v_c;
          /*cout << "Kernel of matrix W:" << endl;
            cout << kerW.t() << endl;*/
     }

   else
    cout << "No appropriate TDA " << endl;
     //   cout << "Residual: " << (W*tau - w).t() << fixed << endl;

    cout << "check constraints :" << endl;
   for(int i=0;i<n;++i)
        cout << "   " << -d[i+n] << " < " << tau[i] << " < " << d[i] << std::endl;
  
    update_d = dTau_max;
    return tau;
}

