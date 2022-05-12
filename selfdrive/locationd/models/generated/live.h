#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1433507162497101333);
void live_err_fun(double *nom_x, double *delta_x, double *out_880135611964011229);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2632836707810806859);
void live_H_mod_fun(double *state, double *out_3075242694947234878);
void live_f_fun(double *state, double dt, double *out_7355716747998289675);
void live_F_fun(double *state, double dt, double *out_2360108211848946778);
void live_h_4(double *state, double *unused, double *out_779120366486272874);
void live_H_4(double *state, double *unused, double *out_296462895026849728);
void live_h_9(double *state, double *unused, double *out_2577433868249301409);
void live_H_9(double *state, double *unused, double *out_2592398657253229614);
void live_h_10(double *state, double *unused, double *out_7019103000767068209);
void live_H_10(double *state, double *unused, double *out_7914309424152769643);
void live_h_12(double *state, double *unused, double *out_3539218832014160674);
void live_H_12(double *state, double *unused, double *out_7370665418655600764);
void live_h_35(double *state, double *unused, double *out_5225937624213227018);
void live_H_35(double *state, double *unused, double *out_8330515622728937143);
void live_h_32(double *state, double *unused, double *out_7197935176897025873);
void live_H_32(double *state, double *unused, double *out_2529211250521324585);
void live_h_13(double *state, double *unused, double *out_195154909553216126);
void live_H_13(double *state, double *unused, double *out_4381917326809517596);
void live_h_14(double *state, double *unused, double *out_2577433868249301409);
void live_H_14(double *state, double *unused, double *out_2592398657253229614);
void live_h_33(double *state, double *unused, double *out_1824357158927816712);
void live_H_33(double *state, double *unused, double *out_5179958618090079539);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}