/* Produced by CVXGEN, 2017-07-03 09:07:08 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  /* Make this a diagonal PSD matrix, even though it's not diagonal. */
  params.Q[0] = 1.5507979025745755;
  params.Q[10] = 0;
  params.Q[20] = 0;
  params.Q[30] = 0;
  params.Q[40] = 0;
  params.Q[50] = 0;
  params.Q[60] = 0;
  params.Q[70] = 0;
  params.Q[80] = 0;
  params.Q[90] = 0;
  params.Q[1] = 0;
  params.Q[11] = 1.7081478226181048;
  params.Q[21] = 0;
  params.Q[31] = 0;
  params.Q[41] = 0;
  params.Q[51] = 0;
  params.Q[61] = 0;
  params.Q[71] = 0;
  params.Q[81] = 0;
  params.Q[91] = 0;
  params.Q[2] = 0;
  params.Q[12] = 0;
  params.Q[22] = 1.2909047389129444;
  params.Q[32] = 0;
  params.Q[42] = 0;
  params.Q[52] = 0;
  params.Q[62] = 0;
  params.Q[72] = 0;
  params.Q[82] = 0;
  params.Q[92] = 0;
  params.Q[3] = 0;
  params.Q[13] = 0;
  params.Q[23] = 0;
  params.Q[33] = 1.510827605197663;
  params.Q[43] = 0;
  params.Q[53] = 0;
  params.Q[63] = 0;
  params.Q[73] = 0;
  params.Q[83] = 0;
  params.Q[93] = 0;
  params.Q[4] = 0;
  params.Q[14] = 0;
  params.Q[24] = 0;
  params.Q[34] = 0;
  params.Q[44] = 1.8929469543476547;
  params.Q[54] = 0;
  params.Q[64] = 0;
  params.Q[74] = 0;
  params.Q[84] = 0;
  params.Q[94] = 0;
  params.Q[5] = 0;
  params.Q[15] = 0;
  params.Q[25] = 0;
  params.Q[35] = 0;
  params.Q[45] = 0;
  params.Q[55] = 1.896293088933438;
  params.Q[65] = 0;
  params.Q[75] = 0;
  params.Q[85] = 0;
  params.Q[95] = 0;
  params.Q[6] = 0;
  params.Q[16] = 0;
  params.Q[26] = 0;
  params.Q[36] = 0;
  params.Q[46] = 0;
  params.Q[56] = 0;
  params.Q[66] = 1.1255853104638363;
  params.Q[76] = 0;
  params.Q[86] = 0;
  params.Q[96] = 0;
  params.Q[7] = 0;
  params.Q[17] = 0;
  params.Q[27] = 0;
  params.Q[37] = 0;
  params.Q[47] = 0;
  params.Q[57] = 0;
  params.Q[67] = 0;
  params.Q[77] = 1.2072428781381868;
  params.Q[87] = 0;
  params.Q[97] = 0;
  params.Q[8] = 0;
  params.Q[18] = 0;
  params.Q[28] = 0;
  params.Q[38] = 0;
  params.Q[48] = 0;
  params.Q[58] = 0;
  params.Q[68] = 0;
  params.Q[78] = 0;
  params.Q[88] = 1.0514672033008299;
  params.Q[98] = 0;
  params.Q[9] = 0;
  params.Q[19] = 0;
  params.Q[29] = 0;
  params.Q[39] = 0;
  params.Q[49] = 0;
  params.Q[59] = 0;
  params.Q[69] = 0;
  params.Q[79] = 0;
  params.Q[89] = 0;
  params.Q[99] = 1.4408098436506365;
  params.c[0] = -1.8804951564857322;
  params.c[1] = -0.17266710242115568;
  params.c[2] = 0.596576190459043;
  params.c[3] = -0.8860508694080989;
  params.c[4] = 0.7050196079205251;
  params.c[5] = 0.3634512696654033;
  params.c[6] = -1.9040724704913385;
  params.c[7] = 0.23541635196352795;
  params.c[8] = -0.9629902123701384;
  params.c[9] = -0.3395952119597214;
  params.A[0] = -0.865899672914725;
  params.A[1] = 0.7725516732519853;
  params.A[2] = -0.23818512931704205;
  params.A[3] = -1.372529046100147;
  params.A[4] = 0.17859607212737894;
  params.A[5] = 1.1212590580454682;
  params.A[6] = -0.774545870495281;
  params.A[7] = -1.1121684642712744;
  params.A[8] = -0.44811496977740495;
  params.A[9] = 1.7455345994417217;
  params.A[10] = 1.9039816898917352;
  params.A[11] = 0.6895347036512547;
  params.A[12] = 1.6113364341535923;
  params.A[13] = 1.383003485172717;
  params.A[14] = -0.48802383468444344;
  params.A[15] = -1.631131964513103;
  params.A[16] = 0.6136436100941447;
  params.A[17] = 0.2313630495538037;
  params.A[18] = -0.5537409477496875;
  params.A[19] = -1.0997819806406723;
  params.A[20] = -0.3739203344950055;
  params.A[21] = -0.12423900520332376;
  params.A[22] = -0.923057686995755;
  params.A[23] = -0.8328289030982696;
  params.A[24] = -0.16925440270808823;
  params.A[25] = 1.442135651787706;
  params.A[26] = 0.34501161787128565;
  params.A[27] = -0.8660485502711608;
  params.A[28] = -0.8880899735055947;
  params.A[29] = -0.1815116979122129;
  params.A[30] = -1.17835862158005;
  params.A[31] = -1.1944851558277074;
  params.A[32] = 0.05614023926976763;
  params.A[33] = -1.6510825248767813;
  params.A[34] = -0.06565787059365391;
  params.A[35] = -0.5512951504486665;
  params.A[36] = 0.8307464872626844;
  params.A[37] = 0.9869848924080182;
  params.A[38] = 0.7643716874230573;
  params.A[39] = 0.7567216550196565;
  params.A[40] = -0.5055995034042868;
  params.A[41] = 0.6725392189410702;
  params.A[42] = -0.6406053441727284;
  params.A[43] = 0.29117547947550015;
  params.A[44] = -0.6967713677405021;
  params.A[45] = -0.21941980294587182;
  params.A[46] = -1.753884276680243;
  params.A[47] = -1.0292983112626475;
  params.A[48] = 1.8864104246942706;
  params.A[49] = -1.077663182579704;
  params.A[50] = 0.7659100437893209;
  params.A[51] = 0.6019074328549583;
  params.A[52] = 0.8957565577499285;
  params.A[53] = -0.09964555746227477;
  params.A[54] = 0.38665509840745127;
  params.A[55] = -1.7321223042686946;
  params.A[56] = -1.7097514487110663;
  params.A[57] = -1.2040958948116867;
  params.A[58] = -1.3925560119658358;
  params.A[59] = -1.5995826216742213;
  params.b[0] = -1.4828245415645833;
  params.b[1] = 0.21311092723061398;
  params.b[2] = -1.248740700304487;
  params.b[3] = 1.808404972124833;
  params.b[4] = 0.7264471152297065;
  params.b[5] = 0.16407869343908477;
}
