#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_5725960303758784919);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9099759843591357967);
void car_H_mod_fun(double *state, double *out_8277430899554790435);
void car_f_fun(double *state, double dt, double *out_6568896444764687930);
void car_F_fun(double *state, double dt, double *out_5349774332356212740);
void car_h_25(double *state, double *unused, double *out_6448750520039690938);
void car_H_25(double *state, double *unused, double *out_6111445790156738774);
void car_h_24(double *state, double *unused, double *out_6740088234593418172);
void car_H_24(double *state, double *unused, double *out_2129122638758970876);
void car_h_30(double *state, double *unused, double *out_1314514736780981503);
void car_H_30(double *state, double *unused, double *out_5418607942061196087);
void car_h_26(double *state, double *unused, double *out_6316188004441827148);
void car_H_26(double *state, double *unused, double *out_2369942471282682550);
void car_h_27(double *state, double *unused, double *out_3032477230390620922);
void car_H_27(double *state, double *unused, double *out_7593371253861620998);
void car_h_29(double *state, double *unused, double *out_3307671292675126811);
void car_H_29(double *state, double *unused, double *out_9140010092978379585);
void car_h_28(double *state, double *unused, double *out_8910762717949336010);
void car_H_28(double *state, double *unused, double *out_4057611075908849011);
void car_h_31(double *state, double *unused, double *out_5830879513698726375);
void car_H_31(double *state, double *unused, double *out_6142091752033699202);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}