#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                       Code generated with sympy 1.9                        *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5725960303758784919) {
   out_5725960303758784919[0] = delta_x[0] + nom_x[0];
   out_5725960303758784919[1] = delta_x[1] + nom_x[1];
   out_5725960303758784919[2] = delta_x[2] + nom_x[2];
   out_5725960303758784919[3] = delta_x[3] + nom_x[3];
   out_5725960303758784919[4] = delta_x[4] + nom_x[4];
   out_5725960303758784919[5] = delta_x[5] + nom_x[5];
   out_5725960303758784919[6] = delta_x[6] + nom_x[6];
   out_5725960303758784919[7] = delta_x[7] + nom_x[7];
   out_5725960303758784919[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9099759843591357967) {
   out_9099759843591357967[0] = -nom_x[0] + true_x[0];
   out_9099759843591357967[1] = -nom_x[1] + true_x[1];
   out_9099759843591357967[2] = -nom_x[2] + true_x[2];
   out_9099759843591357967[3] = -nom_x[3] + true_x[3];
   out_9099759843591357967[4] = -nom_x[4] + true_x[4];
   out_9099759843591357967[5] = -nom_x[5] + true_x[5];
   out_9099759843591357967[6] = -nom_x[6] + true_x[6];
   out_9099759843591357967[7] = -nom_x[7] + true_x[7];
   out_9099759843591357967[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_8277430899554790435) {
   out_8277430899554790435[0] = 1.0;
   out_8277430899554790435[1] = 0;
   out_8277430899554790435[2] = 0;
   out_8277430899554790435[3] = 0;
   out_8277430899554790435[4] = 0;
   out_8277430899554790435[5] = 0;
   out_8277430899554790435[6] = 0;
   out_8277430899554790435[7] = 0;
   out_8277430899554790435[8] = 0;
   out_8277430899554790435[9] = 0;
   out_8277430899554790435[10] = 1.0;
   out_8277430899554790435[11] = 0;
   out_8277430899554790435[12] = 0;
   out_8277430899554790435[13] = 0;
   out_8277430899554790435[14] = 0;
   out_8277430899554790435[15] = 0;
   out_8277430899554790435[16] = 0;
   out_8277430899554790435[17] = 0;
   out_8277430899554790435[18] = 0;
   out_8277430899554790435[19] = 0;
   out_8277430899554790435[20] = 1.0;
   out_8277430899554790435[21] = 0;
   out_8277430899554790435[22] = 0;
   out_8277430899554790435[23] = 0;
   out_8277430899554790435[24] = 0;
   out_8277430899554790435[25] = 0;
   out_8277430899554790435[26] = 0;
   out_8277430899554790435[27] = 0;
   out_8277430899554790435[28] = 0;
   out_8277430899554790435[29] = 0;
   out_8277430899554790435[30] = 1.0;
   out_8277430899554790435[31] = 0;
   out_8277430899554790435[32] = 0;
   out_8277430899554790435[33] = 0;
   out_8277430899554790435[34] = 0;
   out_8277430899554790435[35] = 0;
   out_8277430899554790435[36] = 0;
   out_8277430899554790435[37] = 0;
   out_8277430899554790435[38] = 0;
   out_8277430899554790435[39] = 0;
   out_8277430899554790435[40] = 1.0;
   out_8277430899554790435[41] = 0;
   out_8277430899554790435[42] = 0;
   out_8277430899554790435[43] = 0;
   out_8277430899554790435[44] = 0;
   out_8277430899554790435[45] = 0;
   out_8277430899554790435[46] = 0;
   out_8277430899554790435[47] = 0;
   out_8277430899554790435[48] = 0;
   out_8277430899554790435[49] = 0;
   out_8277430899554790435[50] = 1.0;
   out_8277430899554790435[51] = 0;
   out_8277430899554790435[52] = 0;
   out_8277430899554790435[53] = 0;
   out_8277430899554790435[54] = 0;
   out_8277430899554790435[55] = 0;
   out_8277430899554790435[56] = 0;
   out_8277430899554790435[57] = 0;
   out_8277430899554790435[58] = 0;
   out_8277430899554790435[59] = 0;
   out_8277430899554790435[60] = 1.0;
   out_8277430899554790435[61] = 0;
   out_8277430899554790435[62] = 0;
   out_8277430899554790435[63] = 0;
   out_8277430899554790435[64] = 0;
   out_8277430899554790435[65] = 0;
   out_8277430899554790435[66] = 0;
   out_8277430899554790435[67] = 0;
   out_8277430899554790435[68] = 0;
   out_8277430899554790435[69] = 0;
   out_8277430899554790435[70] = 1.0;
   out_8277430899554790435[71] = 0;
   out_8277430899554790435[72] = 0;
   out_8277430899554790435[73] = 0;
   out_8277430899554790435[74] = 0;
   out_8277430899554790435[75] = 0;
   out_8277430899554790435[76] = 0;
   out_8277430899554790435[77] = 0;
   out_8277430899554790435[78] = 0;
   out_8277430899554790435[79] = 0;
   out_8277430899554790435[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6568896444764687930) {
   out_6568896444764687930[0] = state[0];
   out_6568896444764687930[1] = state[1];
   out_6568896444764687930[2] = state[2];
   out_6568896444764687930[3] = state[3];
   out_6568896444764687930[4] = state[4];
   out_6568896444764687930[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6568896444764687930[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6568896444764687930[7] = state[7];
   out_6568896444764687930[8] = state[8];
}
void F_fun(double *state, double dt, double *out_5349774332356212740) {
   out_5349774332356212740[0] = 1;
   out_5349774332356212740[1] = 0;
   out_5349774332356212740[2] = 0;
   out_5349774332356212740[3] = 0;
   out_5349774332356212740[4] = 0;
   out_5349774332356212740[5] = 0;
   out_5349774332356212740[6] = 0;
   out_5349774332356212740[7] = 0;
   out_5349774332356212740[8] = 0;
   out_5349774332356212740[9] = 0;
   out_5349774332356212740[10] = 1;
   out_5349774332356212740[11] = 0;
   out_5349774332356212740[12] = 0;
   out_5349774332356212740[13] = 0;
   out_5349774332356212740[14] = 0;
   out_5349774332356212740[15] = 0;
   out_5349774332356212740[16] = 0;
   out_5349774332356212740[17] = 0;
   out_5349774332356212740[18] = 0;
   out_5349774332356212740[19] = 0;
   out_5349774332356212740[20] = 1;
   out_5349774332356212740[21] = 0;
   out_5349774332356212740[22] = 0;
   out_5349774332356212740[23] = 0;
   out_5349774332356212740[24] = 0;
   out_5349774332356212740[25] = 0;
   out_5349774332356212740[26] = 0;
   out_5349774332356212740[27] = 0;
   out_5349774332356212740[28] = 0;
   out_5349774332356212740[29] = 0;
   out_5349774332356212740[30] = 1;
   out_5349774332356212740[31] = 0;
   out_5349774332356212740[32] = 0;
   out_5349774332356212740[33] = 0;
   out_5349774332356212740[34] = 0;
   out_5349774332356212740[35] = 0;
   out_5349774332356212740[36] = 0;
   out_5349774332356212740[37] = 0;
   out_5349774332356212740[38] = 0;
   out_5349774332356212740[39] = 0;
   out_5349774332356212740[40] = 1;
   out_5349774332356212740[41] = 0;
   out_5349774332356212740[42] = 0;
   out_5349774332356212740[43] = 0;
   out_5349774332356212740[44] = 0;
   out_5349774332356212740[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_5349774332356212740[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_5349774332356212740[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5349774332356212740[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_5349774332356212740[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_5349774332356212740[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_5349774332356212740[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_5349774332356212740[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_5349774332356212740[53] = -9.8000000000000007*dt;
   out_5349774332356212740[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_5349774332356212740[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_5349774332356212740[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5349774332356212740[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5349774332356212740[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_5349774332356212740[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_5349774332356212740[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_5349774332356212740[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_5349774332356212740[62] = 0;
   out_5349774332356212740[63] = 0;
   out_5349774332356212740[64] = 0;
   out_5349774332356212740[65] = 0;
   out_5349774332356212740[66] = 0;
   out_5349774332356212740[67] = 0;
   out_5349774332356212740[68] = 0;
   out_5349774332356212740[69] = 0;
   out_5349774332356212740[70] = 1;
   out_5349774332356212740[71] = 0;
   out_5349774332356212740[72] = 0;
   out_5349774332356212740[73] = 0;
   out_5349774332356212740[74] = 0;
   out_5349774332356212740[75] = 0;
   out_5349774332356212740[76] = 0;
   out_5349774332356212740[77] = 0;
   out_5349774332356212740[78] = 0;
   out_5349774332356212740[79] = 0;
   out_5349774332356212740[80] = 1;
}
void h_25(double *state, double *unused, double *out_6448750520039690938) {
   out_6448750520039690938[0] = state[6];
}
void H_25(double *state, double *unused, double *out_6111445790156738774) {
   out_6111445790156738774[0] = 0;
   out_6111445790156738774[1] = 0;
   out_6111445790156738774[2] = 0;
   out_6111445790156738774[3] = 0;
   out_6111445790156738774[4] = 0;
   out_6111445790156738774[5] = 0;
   out_6111445790156738774[6] = 1;
   out_6111445790156738774[7] = 0;
   out_6111445790156738774[8] = 0;
}
void h_24(double *state, double *unused, double *out_6740088234593418172) {
   out_6740088234593418172[0] = state[4];
   out_6740088234593418172[1] = state[5];
}
void H_24(double *state, double *unused, double *out_2129122638758970876) {
   out_2129122638758970876[0] = 0;
   out_2129122638758970876[1] = 0;
   out_2129122638758970876[2] = 0;
   out_2129122638758970876[3] = 0;
   out_2129122638758970876[4] = 1;
   out_2129122638758970876[5] = 0;
   out_2129122638758970876[6] = 0;
   out_2129122638758970876[7] = 0;
   out_2129122638758970876[8] = 0;
   out_2129122638758970876[9] = 0;
   out_2129122638758970876[10] = 0;
   out_2129122638758970876[11] = 0;
   out_2129122638758970876[12] = 0;
   out_2129122638758970876[13] = 0;
   out_2129122638758970876[14] = 1;
   out_2129122638758970876[15] = 0;
   out_2129122638758970876[16] = 0;
   out_2129122638758970876[17] = 0;
}
void h_30(double *state, double *unused, double *out_1314514736780981503) {
   out_1314514736780981503[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5418607942061196087) {
   out_5418607942061196087[0] = 0;
   out_5418607942061196087[1] = 0;
   out_5418607942061196087[2] = 0;
   out_5418607942061196087[3] = 0;
   out_5418607942061196087[4] = 1;
   out_5418607942061196087[5] = 0;
   out_5418607942061196087[6] = 0;
   out_5418607942061196087[7] = 0;
   out_5418607942061196087[8] = 0;
}
void h_26(double *state, double *unused, double *out_6316188004441827148) {
   out_6316188004441827148[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2369942471282682550) {
   out_2369942471282682550[0] = 0;
   out_2369942471282682550[1] = 0;
   out_2369942471282682550[2] = 0;
   out_2369942471282682550[3] = 0;
   out_2369942471282682550[4] = 0;
   out_2369942471282682550[5] = 0;
   out_2369942471282682550[6] = 0;
   out_2369942471282682550[7] = 1;
   out_2369942471282682550[8] = 0;
}
void h_27(double *state, double *unused, double *out_3032477230390620922) {
   out_3032477230390620922[0] = state[3];
}
void H_27(double *state, double *unused, double *out_7593371253861620998) {
   out_7593371253861620998[0] = 0;
   out_7593371253861620998[1] = 0;
   out_7593371253861620998[2] = 0;
   out_7593371253861620998[3] = 1;
   out_7593371253861620998[4] = 0;
   out_7593371253861620998[5] = 0;
   out_7593371253861620998[6] = 0;
   out_7593371253861620998[7] = 0;
   out_7593371253861620998[8] = 0;
}
void h_29(double *state, double *unused, double *out_3307671292675126811) {
   out_3307671292675126811[0] = state[1];
}
void H_29(double *state, double *unused, double *out_9140010092978379585) {
   out_9140010092978379585[0] = 0;
   out_9140010092978379585[1] = 1;
   out_9140010092978379585[2] = 0;
   out_9140010092978379585[3] = 0;
   out_9140010092978379585[4] = 0;
   out_9140010092978379585[5] = 0;
   out_9140010092978379585[6] = 0;
   out_9140010092978379585[7] = 0;
   out_9140010092978379585[8] = 0;
}
void h_28(double *state, double *unused, double *out_8910762717949336010) {
   out_8910762717949336010[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4057611075908849011) {
   out_4057611075908849011[0] = 1;
   out_4057611075908849011[1] = 0;
   out_4057611075908849011[2] = 0;
   out_4057611075908849011[3] = 0;
   out_4057611075908849011[4] = 0;
   out_4057611075908849011[5] = 0;
   out_4057611075908849011[6] = 0;
   out_4057611075908849011[7] = 0;
   out_4057611075908849011[8] = 0;
}
void h_31(double *state, double *unused, double *out_5830879513698726375) {
   out_5830879513698726375[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6142091752033699202) {
   out_6142091752033699202[0] = 0;
   out_6142091752033699202[1] = 0;
   out_6142091752033699202[2] = 0;
   out_6142091752033699202[3] = 0;
   out_6142091752033699202[4] = 0;
   out_6142091752033699202[5] = 0;
   out_6142091752033699202[6] = 0;
   out_6142091752033699202[7] = 0;
   out_6142091752033699202[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5725960303758784919) {
  err_fun(nom_x, delta_x, out_5725960303758784919);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9099759843591357967) {
  inv_err_fun(nom_x, true_x, out_9099759843591357967);
}
void car_H_mod_fun(double *state, double *out_8277430899554790435) {
  H_mod_fun(state, out_8277430899554790435);
}
void car_f_fun(double *state, double dt, double *out_6568896444764687930) {
  f_fun(state,  dt, out_6568896444764687930);
}
void car_F_fun(double *state, double dt, double *out_5349774332356212740) {
  F_fun(state,  dt, out_5349774332356212740);
}
void car_h_25(double *state, double *unused, double *out_6448750520039690938) {
  h_25(state, unused, out_6448750520039690938);
}
void car_H_25(double *state, double *unused, double *out_6111445790156738774) {
  H_25(state, unused, out_6111445790156738774);
}
void car_h_24(double *state, double *unused, double *out_6740088234593418172) {
  h_24(state, unused, out_6740088234593418172);
}
void car_H_24(double *state, double *unused, double *out_2129122638758970876) {
  H_24(state, unused, out_2129122638758970876);
}
void car_h_30(double *state, double *unused, double *out_1314514736780981503) {
  h_30(state, unused, out_1314514736780981503);
}
void car_H_30(double *state, double *unused, double *out_5418607942061196087) {
  H_30(state, unused, out_5418607942061196087);
}
void car_h_26(double *state, double *unused, double *out_6316188004441827148) {
  h_26(state, unused, out_6316188004441827148);
}
void car_H_26(double *state, double *unused, double *out_2369942471282682550) {
  H_26(state, unused, out_2369942471282682550);
}
void car_h_27(double *state, double *unused, double *out_3032477230390620922) {
  h_27(state, unused, out_3032477230390620922);
}
void car_H_27(double *state, double *unused, double *out_7593371253861620998) {
  H_27(state, unused, out_7593371253861620998);
}
void car_h_29(double *state, double *unused, double *out_3307671292675126811) {
  h_29(state, unused, out_3307671292675126811);
}
void car_H_29(double *state, double *unused, double *out_9140010092978379585) {
  H_29(state, unused, out_9140010092978379585);
}
void car_h_28(double *state, double *unused, double *out_8910762717949336010) {
  h_28(state, unused, out_8910762717949336010);
}
void car_H_28(double *state, double *unused, double *out_4057611075908849011) {
  H_28(state, unused, out_4057611075908849011);
}
void car_h_31(double *state, double *unused, double *out_5830879513698726375) {
  h_31(state, unused, out_5830879513698726375);
}
void car_H_31(double *state, double *unused, double *out_6142091752033699202) {
  H_31(state, unused, out_6142091752033699202);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
