/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

//#include <examples/lqr_control/trial_lqr_control.hpp> //change later
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
//#include <lib/geo/geo.h>
#include <circuit_breaker/circuit_breaker.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <memory>
#include <sstream>
#include <poll.h>
#include <numeric>

using namespace matrix;
using namespace time_literals;
using namespace std;

float a = 3.0 ;//trajectory size
float z0 = 3.0 ;//initial height
float m = 0.8;
float g = 9.81;
int steps = 2514.0;
float Ts = 0.0100; // uncommented
float x = 0.0;
float xdot = 0.0;
float y = 0.0;
float ydot= 0.0;
float z;
float zdot;
float psi ;
float gam = m*g;
float sig = 0.0;
int i = 0;


// float ubar[4][1] {};
// ubar[0][0] = 0.0;
// ubar[1][0] = 0.0;
// ubar[2][0] = 0.0;
// ubar[3][0] = 0.0;

// float u = ubar

//TODO current x matrix size 12 by 1  <=====================


// float x0[14][1] {};
// x0[0][0] = a
// x0[1][0] = 0.0
// x0[2][0] = z0
// x0[3][0] = 0.0
// x0[4][0] = 0
// x0[5][0] = 0
// x0[6][0] = 0
// x0[7][0] = 0
// x0[8][0] = 0
// x0[9][0] = 0
// x0[10][0] = 0
// x0[11][0] = 0
// x0[12][0] = m*g
// x0[13][0] = 0

// TODO: populate state
// need x y z, phi theta psi, xdot ydot zdot, p q r
// Also need gamma and sig
// Initialize gamma = ubar, sig = 0
// Then update with gamma = thrust, sig = d/dt gamma

/*
Outside of the loop
    1) Define z_ref. Matrix, size 14 * steps. TODO (done)
        Diff equivalent: [0, x[1:steps-1] - x[0:steps-2]]/Ts

        Hover: x and derivatives, y and derivatives, are zero
        z linearly spaced to zmax, then held
        dz constant, then zero
    2) Initialize states, initialize gamma = mass*g, sigma = 0
    3) Initialize z_eval, delta_mat, b_vec, alpha, beta, v, u_control

Inside each loop:
    1) Get the state xi [x y z, phi theta psi, xdot ydot zdot, p q r, gamma sig] TODO
        First 12 states are coming from sensor readings
        gamma += Ts*sig
        sig += Ts*u[0]
    2) Use state to compute z_eval, delta_mat, and b_vec
    3) Use those to compute alpha, beta
    4) Get our error = z_eval - z_ref[i]
    5) v = -K * error
    6) u_control = alpha + beta * v [T'' taux tauy tauz]

*/


float k = 2.0
float loops = 2.0
float tstart = 0.0
float ttend = 2*M_PI*k*loops
float Ts = 0.0100;
template <typename T>
std::vector<T> linspace(double start, double end, double num)
{
    std::vector<T> tt;

    if (0 != num)
    {
        if (1 == num)
        {
            tt.push_back(static_cast<T>(start));
        }
        else
        {
            double delta = (end - start) / (num - 1);

            for (auto i = 0; i < (num - 1); ++i)
            {
                tt.push_back(static_cast<T>(start + delta * i));
            }
            // ensure that start and end are exactly the same as the input
            tt.push_back(static_cast<T>(end));
        }
    }
    return tt;
}

float xref[steps] = {}; // zeros
float yref[steps] = {};
float zref[steps];
std::fill_n(zref, steps, z0); //constant height
float phiref[steps] = {};

//float xref = a*cos(tt/k)./(1+sin(tt/k).^2);
//float yref = a*cos(tt/k).*sin(tt/k)./(1+sin(tt/k).^2);
//float zref = z0 * ones(size(xref));
//float phiref = zeros(size(xref));

float zTraj_x = xref;
float zTraj_y = yref;
float zTraj_z = zref;
float zTraj_psi = psiref;

float zTraj_dx[steps];
std::adjacent_difference(zTraj_x, zTraj_x + steps, zTraj_dx);
zTraj_dx = zTraj_dx/Ts;

float zTraj_ddx[steps];
std::adjacent_difference(zTraj_dx, zTraj_dx + steps, zTraj_ddx);
zTraj_ddx = zTraj_ddx/Ts;

float zTraj_dddx[steps];
std::adjacent_difference(zTraj_ddx, zTraj_ddx + steps, zTraj_dddx);
zTraj_dddx = zTraj_dddx/Ts;

float zTraj_dy[steps];
std::adjacent_difference(zTraj_y, zTraj_y + steps, zTraj_dy);
zTraj_dy = zTraj_dy/Ts;

float zTraj_ddy[steps];
std::adjacent_difference(zTraj_dy, zTraj_dy + steps, zTraj_ddy);
zTraj_ddy = zTraj_ddy/Ts;

float zTraj_dddy[steps];
std::adjacent_difference(zTraj_ddy, zTraj_ddy + steps, zTraj_dddy);
zTraj_dddy = zTraj_dddy/Ts;

float zTraj_dz[steps];
std::adjacent_difference(zTraj_z, zTraj_z + steps, zTraj_dz);
zTraj_dz = zTraj_dz/Ts;

float zTraj_ddz[steps];
std::adjacent_difference(zTraj_dz, zTraj_dz + steps, zTraj_ddz);
zTraj_ddz = zTraj_ddz/Ts;

float zTraj_dddz[steps];
std::adjacent_difference(zTraj_ddz, zTraj_ddz + steps, zTraj_dddz);
zTraj_dddz = zTraj_dddz/Ts;

float zTraj_dpsi[steps];
std::adjacent_difference(zTraj_psi, zTraj_psi + steps, zTraj_dpsi);
zTraj_dpsi = zTraj_dpsi/Ts;



Matrix<float,14,2514> zTraj_full
zTraj_full [0][:] = zTraj_x
zTraj_full [1][:] = zTraj_dx
zTraj_full [2][:] = zTraj_ddx
zTraj_full [3][:] = zTraj_dddx
zTraj_full [4][:] = zTraj_y
zTraj_full [5][:] = zTraj_dy
zTraj_full [6][:] = zTraj_ddy
zTraj_full [7][:] = zTraj_dddy
zTraj_full [8][:] = zTraj_z
zTraj_full [9][:] = zTraj_dz
zTraj_full [10][:] = zTraj_ddz
zTraj_full [11][:] = zTraj_dddz
zTraj_full [12][:] = zTraj_psi
zTraj_full [13][:] = zTraj_dpsi


Matrix<float,14,steps> z_ref;
Matrix<float,4,4> beta;
Matrix<float,4,1> alpha;
Matrix<float,4,1> u;
Matrix <float,4,1> ubar;
ubar[0][0] = 0
ubar[1][0] = 0
ubar[2][0] = 0
ubar[3][0] = 0

Matrix<float,14,steps>;
Matrix <float,14,4> z_eval; // this stays outside the loop




z_eval[0][0] = x
z_eval[1][0] = xdot
z_eval[2][0] =  -(5.0f*gam*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/4.0f
z_eval[3][0] = (5.0f*gam*p*cos(psi)*sin(phi)*sin(theta))/4.0f - (5.0f*sig*cos(phi)*cos(psi)*sin(theta))/4 .0f- (5.0f*gam*q*cos(psi)*cos(theta))/4.0f - (5.0f*gam*p*cos(phi)*sin(psi))/4.0f - (5.0f*sig*sin(phi)*sin(psi))/4.0f
z_eval[4][0] =  y
z_eval[5][0] =  ydot
z_eval[6][0] = (5.0f*gam*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/4.0f
z_eval[7][0] = (5.0f*sig*cos(psi)*sin(phi))/4.0f - (5.0f*sig*cos(phi)*sin(psi)*sin(theta))/4.0f + (5.0f*gam*p*cos(phi)*cos(psi))/4.0f - (5.0f*gam*q*cos(theta)*sin(psi))/4.0f + (5.0f*gam*p*sin(phi)*sin(psi)*sin(theta))/4.0f
z_eval[8][0] = z
z_eval[9][0] = zdot
z_eval[10][0] = 9.81f - (5.0f*gam*cos(phi)*cos(theta))/4.0f //changed
z_eval[11][0] = (5.0f*gam*q*sin(theta))/4.0f - (5.0f*sig*cos(phi)*cos(theta))/4.0f + (5.0f*gam*p*cos(theta)*sin(phi))/4.0f
z_eval[12][0] = psi
z_eval[13][0] =  (r*cos(phi) + q*sin(phi))/cos(theta)


// add to all values add .0f
Matrix <float,4,4> delta_mat;
delta_mat[0][0] = - (5.0f*sin(phi)*sin(psi))/4.0f - (5.0f*cos(phi)*cos(psi)*sin(theta))/4.0f
delta_mat[0][1] = 250.0f*gam*cos(psi)*sin(phi)*sin(theta) - 250.0f*gam*cos(phi)*sin(psi)
delta_mat[0][2] = -250.0f*gam*cos(psi)*cos(theta)
delta_mat[0][3] = 0.0f

delta_mat[1][0] = (5.0f*cos(psi)*sin(phi))/4.0f - (5.0f*cos(phi)*sin(psi)*sin(theta))/4.0f
delta_mat[1][1] = 250.0f*gam*cos(phi)*cos(psi) + 250.0f*gam*sin(phi)*sin(psi)*sin(theta)
delta_mat[1][2] = -250.0f*gam*cos(theta)*sin(psi)
delta_mat[1][3] = 0.0f

delta_mat[2][0] =  -(5.0f*cos(phi)*cos(theta))/4.0f
delta_mat[2][1] = 250.0f*gam*cos(theta)*sin(phi)
delta_mat[2][2] = 250.0f*gam*sin(theta)
delta_mat[2][3] = 0.0f

delta_mat[3][0] = 0.0f
delta_mat[3][1] = 0.0f
delta_mat[3][2] = (200.0f*sin(phi))/cos(theta)
delta_mat[3][3] = (1000.0f*cos(phi))/(9.0f*cos(theta))


Matrix <float,4,1> b_vec;

b_vec[0][0] = (5.0f*gam*p^2*sin(phi)*sin(psi))/4.0f + (5.0f*gam*q^2*sin(phi)*sin(psi))/4.0f - (5.0f*q*sig*cos(psi)*cos(theta))/2.0f - (5.0f*p*sig*cos(phi)*sin(psi))/2.0f + (5.0f*gam*p^2*cos(phi)*cos(psi)*sin(theta))/4.0f + (5.0f*gam*q^2*cos(phi)*cos(psi)*sin(theta))/4.0f - (9.0f*gam*p*r*cos(psi)*cos(theta))/4.0f + (9.0f*gam*q*r*cos(phi)*sin(psi))/4.0f + (5.0f*p*sig*cos(psi)*sin(phi)*sin(theta))/2.0f - (9.0f*gam*q*r*cos(psi)*sin(phi)*sin(theta))/4.0f
// Changed 2.0f to 2 in exponent
b_vec[1][0] = (5.0f*p*sig*cos(phi)*cos(psi))/2.0f - (5.0f*gam*q^2*cos(psi)*sin(phi))/4.0f - (5.0f*gam*p^2*cos(psi)*sin(phi))/4.0f - (5.0f*q*sig*cos(theta)*sin(psi))/2.0f + (5.0f*gam*p^2*cos(phi)*sin(psi)*sin(theta))/4.0f + (5.0f*gam*q^2*cos(phi)*sin(psi)*sin(theta))/4.0f - (9.0f*gam*q*r*cos(phi)*cos(psi))/4.0f - (9.0f*gam*p*r*cos(theta)*sin(psi))/4.0f + (5.0f*p*sig*sin(phi)*sin(psi)*sin(theta))/2.0f - (9.0f*gam*q*r*sin(phi)*sin(psi)*sin(theta))/4.0f
b_vec[2][0] = (5.0f*q*sig*sin(theta))/2.0f + (5.0f*gam*p^2*cos(phi)*cos(theta))/4.0f + (5.0f*gam*q^2*cos(phi)*cos(theta))/4.0f + (9.0f*gam*p*r*sin(theta))/4.0f + (5.0f*p*sig*cos(theta)*sin(phi))/2.0f - (9.0f*gam*q*r*cos(theta)*sin(phi))/4.0f
b_vec[3][0] = -(10.0f*q*r*sin(theta) - 20.0f*q*r*cos(phi)^2*sin(theta) - 10.0f*q^2*cos(phi)*sin(phi)*sin(theta) + 10.0f*r^2*cos(phi)*sin(phi)*sin(theta) - 5.0f*p*q*cos(phi)*cos(theta) + p*r*cos(theta)*sin(phi))/(5.0f*cos(theta)^2)


Matrix <float,4,14> K;
//Matrix K[4][14] {};

K(0,0) = 60.0f
K[0][1] = 92.0f // changed
K[0][2] = 51.0f
K[0][3] = 12.0f
K[0][4] = 0.0f
K[0][5] = 0.0f
K[0][6] = 0.0f
K[0][7] = 0.0f
K[0][8] = 0.0f
K[0][9] = 0.0f
K[0][10] = 0.0f
K[0][11] = 0.0f
K[0][12] = 0.0f
K[0][13] = 0.0f


K[1][0] = 0.0f
K[1][1] = 0.0f
K[1][2] = 0.0f
K[1][3] = 0.0f
K[1][4] = 60.0f
K[1][5] = 92.0f //changed
K[1][6] = 51.0f
K[1][7] = 12.0f
K[1][8] = 0.0f
K[1][9] = 0.0f
K[1][10] = 0.0f
K[1][11] = 0.0f
K[1][12] = 0.0f
K[1][13] = 0.0f

K[2][0] = 0.0f
K[2][1] = 0.0f
K[2][2] = 0.0f
K[2][3] = 0.0f
K[2][4] = 0.0f
K[2][5] = 0.0f
K[2][6] = 0.0f
K[2][7] = 0.0f
K[2][8] = 60.0f //changed
K[2][9] = 92.0f //changed
K[2][10] = 51.0f //changed
K[2][11] = 12.0f
K[2][12] = 0.0f
K[2][13] = 0.0f

K[3][0] = 0.0f
K[3][1] = 0.0f
K[3][2] = 0.0f
K[3][3] = 0.0f
K[3][4] = 0.0f
K[3][5] = 0.0f
K[3][6] = 0.0f
K[3][7] = 0.0f
K[3][8] = 0.0f
K[3][9] = 0.0f
K[3][10] = 0.0f
K[3][11] = 0.0f
K[3][12] = 15.0f
K[3][13] = 8.0f

vectordouble linspace(float a, float b, float num)
{
            a=0.0;
            num = steps;

             // create a vector of length num
             vectordouble v(num);
             double tmp = 0.0;

             //int lala = num;

             // now assign the values to the vector
             for (int i = 0; i < num; i++)
                 {
                      v.x[i] = a + i * ( (b - a) / num );
                 }
             return v;
}

//v is equivalent to v_con (matlab)
alpha = -inv(delta_mat)*b_vec
beta = inv(delta_mat)
v = -K*(z_eval - z_ref); // TODO: find zref
u = alpha + beta*v // [T'' taux tauy tauz]
sigma += Ts*u[0]; // initialize to 0
gamma += Ts*sigma; // initialize to mass*9.81

int MulticopterLqrControl::print_usage(const char *reason)
{
    if (reason){
        cout << "PX4 Warm";
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
                R"DESCR_STR(
### Description

)DESCR_STR");
        //need to change lqr_control to Fb_control and lqr to Fb
    PRINT_MODULE_USAGE_NAME("lqr_control","lqr");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;

}
//change MulticopterLqrControl to MulticopterFbControl
MulticopterLqrControl::MulticopterLqrControl():ModuleParams(nullptr),_loop_perf(perf_alloc(PC_ELAPSED,"lqr_control"))
{

    for (int i=0;i<12;i++)
    {
        _eq_point(i,0) = 0.0f;

        _x(i,0) = 0.0f;
    }

    // _eq_point(5,0) = -1.0f;

    for (int i=0;i<4;i++)
    {
        _u_controls_norm(i,0) = 0.0f;
    }
    memset(&_actuators, 0, sizeof(_actuators));


    _K = readMatrixK("/Applications/PX4-Autopilot/src/examples/lqr_control/lqr_files/new_controller.txt");
//	_P = readMatrixP("/home/zach/Firmare/src/examples/lqr_control/lqr_files/controller/new_pe.txt");

}

// =================================================================================================================
// Read from File

Matrix <float,4,12> MulticopterLqrControl::readMatrixK(const char *filename)
{
    static Matrix <float,4,12> result;

    static int rows = 4;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
        for (int i=0; i<rows;i++){
            string line;
            getline(infile, line);
            stringstream stream(line);
            for (int j=0; j<cols; j++){
                stream >> result(i,j);
            }
        }
        infile.close();
    }else cout << "Unable to open file";
    return result;
}


Matrix <float,12,12> MulticopterLqrControl::readMatrixP(const char *filename)
{
    static Matrix <float,12,12> result;
    static int rows = 12;
    static int cols = 12;
    ifstream infile;
    infile.open(filename);
    if (infile.is_open()){
        for (int i=0; i<rows;i++){
            string line;
            getline(infile, line);
            stringstream stream(line);
            for (int j=0; j<cols; j++){
                stream >> result(i,j);
            }
        }
        infile.close();
    }else cout << "Unable to open file";
    return result;
}


// =================================================================================================================
// Set Current State

void MulticopterLqrControl::setCurrentState()
{
    matrix::Quatf q;
    q = Quatf(_v_att.q);
    q.normalize();

    _x(0,0) = _v_local_pos.x; // x
    _x(1,0) = _v_local_pos.y; // y
    _x(2,0) = -_v_local_pos.z; // z
    _x(3,0) = _v_local_pos.vx; //xdot
    _x(4,0) = _v_local_pos.vy; //ydot
    _x(5,0) = _v_local_pos.vz; //zdot

    _x(6,0) = Eulerf(q).phi(); // phi
    _x(7,0) = Eulerf(q).theta(); // theta
    _x(8,0) = Eulerf(q).psi(); // psi
    _x(9,0) = _v_ang_vel.xyz[0]; // p
    _x(10,0) = _v_ang_vel.xyz[1]; // q
    _x(11,0) = _v_ang_vel.xyz[2]; // r



}

    float x = _x(0,0)
    float y = _x(1,0)
    float z = _x(2,0)
    float xdot = _x(3,0)
    float ydot = _x(4,0)
    float zdot = _x(5,0)
    float phi = _x(6,0)
    float theta = _x(7,0)
    float psi = _x(8,0)
    float p = _x(9,0)
    float q = _x(10,0)
    float r = _x(11,0)


// =================================================================================================================
// Compute Controls

//need to replace compute controls to specific control model

void MulticopterLqrControl::computeControls()
{

    static Matrix<float,4,1> u_control; //used
    static Matrix<float,12,1> delta_x; //used
    //static float dt = (_curr_run - _last_run)/1e6f; #added from gs

    //============ Generate Trajectory ===============
    //add these topics
    //Generate Trajectory

    delta_mat[0][0] = - (5.0f*sin(phi)*sin(psi))/4.0f - (5.0f*cos(phi)*cos(psi)*sin(theta))/4.0f
    delta_mat[0][1] = 250.0f*gam*cos(psi)*sin(phi)*sin(theta) - 250.0f*gam*cos(phi)*sin(psi)
    delta_mat[0][2] = -250.0f*gam*cos(psi)*cos(theta)
    delta_mat[0][3] = 0.0f

    delta_mat[1][0] = (5.0f*cos(psi)*sin(phi))/4.0f - (5.0f*cos(phi)*sin(psi)*sin(theta))/4.0f
    delta_mat[1][1] = 250.0f*gam*cos(phi)*cos(psi) + 250.0f*gam*sin(phi)*sin(psi)*sin(theta)
    delta_mat[1][2] = -250.0f*gam*cos(theta)*sin(psi)
    delta_mat[1][3] = 0.0f

    delta_mat[2][0] =  -(5.0f*cos(phi)*cos(theta))/4.0f
    delta_mat[2][1] = 250.0f*gam*cos(theta)*sin(phi)
    delta_mat[2][2] = 250.0f*gam*sin(theta)
    delta_mat[2][3] = 0.0f

    delta_mat[3][0] = 0.0f
    delta_mat[3][1] = 0.0f
    delta_mat[3][2] = (200.0f*sin(phi))/cos(theta)
    delta_mat[3][3] = (1000.0f*cos(phi))/(9.0f*cos(theta))

    b_vec[0][0] = (5.0f*gam*p^2*sin(phi)*sin(psi))/4.0f + (5.0f*gam*q^2*sin(phi)*sin(psi))/4.0f - (5.0f*q*sig*cos(psi)*cos(theta))/2.0f - (5.0f*p*sig*cos(phi)*sin(psi))/2.0f + (5.0f*gam*p^2*cos(phi)*cos(psi)*sin(theta))/4.0f + (5.0f*gam*q^2*cos(phi)*cos(psi)*sin(theta))/4.0f - (9.0f*gam*p*r*cos(psi)*cos(theta))/4.0f + (9.0f*gam*q*r*cos(phi)*sin(psi))/4.0f + (5.0f*p*sig*cos(psi)*sin(phi)*sin(theta))/2.0f - (9.0f*gam*q*r*cos(psi)*sin(phi)*sin(theta))/4.0f
    b_vec[1][0] = (5.0f*p*sig*cos(phi)*cos(psi))/2.0f - (5.0f*gam*q^2*cos(psi)*sin(phi))/4.0f - (5.0f*gam*p^2*cos(psi)*sin(phi))/4.0f - (5.0f*q*sig*cos(theta)*sin(psi))/2.0f + (5.0f*gam*p^2*cos(phi)*sin(psi)*sin(theta))/4.0f + (5.0f*gam*q^2*cos(phi)*sin(psi)*sin(theta))/4.0f - (9.0f*gam*q*r*cos(phi)*cos(psi))/4.0f - (9.0f*gam*p*r*cos(theta)*sin(psi))/4.0f + (5.0f*p*sig*sin(phi)*sin(psi)*sin(theta))/2.0f - (9.0f*gam*q*r*sin(phi)*sin(psi)*sin(theta))/4.0f
    b_vec[2][0] = (5.0f*q*sig*sin(theta))/2.0f + (5.0f*gam*p^2*cos(phi)*cos(theta))/4.0f + (5.0f*gam*q^2*cos(phi)*cos(theta))/4.0f + (9.0f*gam*p*r*sin(theta))/4.0f + (5.0f*p*sig*cos(theta)*sin(phi))/2.0f - (9.0f*gam*q*r*cos(theta)*sin(phi))/4.0f
    b_vec[3][0] = -(10.0f*q*r*sin(theta) - 20.0f*q*r*cos(phi)^2*sin(theta) - 10.0f*q^2*cos(phi)*sin(phi)*sin(theta) + 10.0f*r^2*cos(phi)*sin(phi)*sin(theta) - 5.0f*p*q*cos(phi)*cos(theta) + p*r*cos(theta)*sin(phi))/(5.0f*cos(theta)^2)


    z_eval[0][0] = x
    z_eval[1][0] = xdot
    z_eval[2][0] =  -(5.0f*gam*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/4.0f
    z_eval[3][0] = (5.0f*gam*p*cos(psi)*sin(phi)*sin(theta))/4.0f - (5.0f*sig*cos(phi)*cos(psi)*sin(theta))/4 .0f- (5.0f*gam*q*cos(psi)*cos(theta))/4.0f - (5.0f*gam*p*cos(phi)*sin(psi))/4.0f - (5.0f*sig*sin(phi)*sin(psi))/4.0f
    z_eval[4][0] =  y
    z_eval[5][0] =  ydot
    z_eval[6][0] = (5.0f*gam*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/4.0f
    z_eval[7][0] = (5.0f*sig*cos(psi)*sin(phi))/4.0f - (5.0f*sig*cos(phi)*sin(psi)*sin(theta))/4.0f + (5.0f*gam*p*cos(phi)*cos(psi))/4.0f - (5.0f*gam*q*cos(theta)*sin(psi))/4.0f + (5.0f*gam*p*sin(phi)*sin(psi)*sin(theta))/4.0f
    z_eval[8][0] = z
    z_eval[9][0] = zdot
    z_eval[10][0] = 9.81f - (5.0f*gam*cos(phi)*cos(theta))/4.0f //changed
    z_eval[11][0] = (5.0f*gam*q*sin(theta))/4.0f - (5.0f*sig*cos(phi)*cos(theta))/4.0f + (5.0f*gam*p*cos(theta)*sin(phi))/4.0f
    z_eval[12][0] = psi
    z_eval[13][0] =  (r*cos(phi) + q*sin(phi))/cos(theta)
    gam += Ts*sig
    sig += Ts*u[0]




    alpha = -inv(delta_mat)*b_vec
    beta = inv(delta_mat)

    error = z_eval - z_ref[:][i]
    i +=1
    //Integrate Trajectory Error
    //Identify Controller
    //Compute Control Input


    _eq_point(0,0) = 0.0;
    _eq_point(1,0) = 0.0;
    _eq_point(2,0) = 1.0;
    _eq_point(3,0) = 0.0;
    _eq_point(4,0) = 0.0;
    _eq_point(5,0) = 0.0;

    _eq_point(6,0) = 0.0;
    _eq_point(7,0) = 0.0;
    _eq_point(8,0) = 0.0;
    _eq_point(9,0) = 0.0;
    _eq_point(10,0) = 0.0;
    _eq_point(11,0) = 0.0;

    //float t = last_run/1e6;

    //_eq_point(0,0) = 0.5f*sin(t);
    //_eq_point(1,0) = 0.5f*0.5f*sin(t*2.0f);
    //cout << _eq_point(0,0);
    //cout << "\n";



    //================================================



    delta_x = _x - _eq_point; // Change to z
    u_control = -_K*(delta_x);
    cout << last_run;
    cout << "\n\n";

    // with full
    _u_controls_norm(1,0) = fmin(fmax((u_control(1,0))/(0.1650f*4.0f), -1.0f), 1.0f);
    _u_controls_norm(2,0) = fmin(fmax((u_control(2,0))/(0.1650f*4.0f), -1.0f), 1.0f);
    _u_controls_norm(3,0) = fmin(fmax((u_control(3,0))/(0.1f*1.0f), -1.0f), 1.0f);
    _u_controls_norm(0,0) = fmin(fmax((u_control(0,0))+ff_thrust/16.0f, 0.0f), 1.0f);


    return;
}


// =================================================================================================================
// Poll EKF States

void MulticopterLqrControl::vehicle_attitude_poll() // phi theta psi
{

    // check if there is a new message
    bool updated;
    orb_check(_v_att_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
    }
    return;
}

void MulticopterLqrControl::vehicle_position_poll() // x y z and velocities
{

    // check if there is a new message
    bool updated;
    orb_check(_v_local_pos_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_local_position), _v_local_pos_sub, &_v_local_pos);
    }
    return;
}

void MulticopterLqrControl::vehicle_angular_velocity_poll() // p q r
{
    // check if there is a new message
    bool updated;
    orb_check(_v_ang_vel_sub, &updated);

    if (updated) {
        orb_copy(ORB_ID(vehicle_angular_velocity), _v_ang_vel_sub, &_v_ang_vel);
    }
    return;
}


// =================================================================================================================
// Write to File

//change MulticopterLqrControl to multicopterFbControl
void MulticopterLqrControl::writeStateOnFile(const char *filename, Matrix <float,12,1>vect, hrt_abstime t) {

    ofstream outfile;
    outfile.open(filename, std::ios::out | std::ios::app);

    outfile << t << "\t"; // time

    for(int i=0;i>12;i++){
        if(i==11){
            outfile << vect(i,0) << "\n";
        }else{
            outfile << vect(i,0) << "\t";
        }
    }
    outfile.close();
    return;
}
//change MulticopterLqrControl to multicopterFbControl
void MulticopterLqrControl::writeActuatorControlsOnFile(const char *filename, Matrix <float,4,1>vect, hrt_abstime t) {

    ofstream outfile;
    outfile.open(filename, std::ios::out | std::ios::app);

    outfile << t << "\t"; // time

    for(int i=0;i>4;i++){
        if(i==3){
            outfile << vect(i,0) << "\n";
        }else{
            outfile << vect(i,0) << "\t";
        }
    }
    outfile.close();
    return;
}


// =================================================================================================================
// Publish Actuator Controls

//change MulticopterLqrControl to multicopterFbControl
void
MulticopterLqrControl::publish_actuator_controls()
{
    _actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
    _actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
    _actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
    _actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;

    _actuators.timestamp = hrt_absolute_time();

    _actuators_id = ORB_ID(actuator_controls_0);
    _actuators_0_pub = orb_advertise(_actuators_id, &_actuators);

    static int _pub = orb_publish(_actuators_id,_actuators_0_pub, &_actuators);

    if (_pub != 0){

        cout << "PX4 Warm";
        PX4_ERR("Publishing actuators fails!");
    }
}

// Run


//change MulticopterLqrControl to multicopterFbControl
void MulticopterLqrControl::run()
{

    // do subscriptions
    _v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _v_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _v_ang_vel_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));


    //change last_run to _last_run
    last_run = hrt_absolute_time();
    cout << "lqr_control RUN, ... \n";

    px4_show_tasks(); // show tasks
    //edit
    while (!should_exit()) {

        perf_begin(_loop_perf);
        _actuators_id = ORB_ID(actuator_controls_0);

        const hrt_abstime now = hrt_absolute_time();
        const float dt = ((now - last_run)/1e6f);
        if (dt>0.010f){
            last_run = now;

            setCurrentState();
            computeControls();

            // publish actuator

            _thrust_sp = _u_controls_norm(0,0);
            _att_control(0) = _u_controls_norm(1,0);
            _att_control(1) = _u_controls_norm(2,0);
            _att_control(2) = _u_controls_norm(3,0);

             publish_actuator_controls();
             // _last_run
        }

        vehicle_attitude_poll();
        vehicle_position_poll();
        vehicle_angular_velocity_poll();
        perf_end(_loop_perf);
    }

    orb_unsubscribe(_v_att_sub);
    orb_unsubscribe(_v_local_pos_sub);
    orb_unsubscribe(_v_ang_vel_sub);
}


int MulticopterLqrControl::task_spawn(int argc, char *argv[])
{									//change lqr_control to Fb_control
    _task_id = px4_task_spawn_cmd("lqr_control",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_DEFAULT+40,
                                    3250,
                                    (px4_main_t)&run_trampoline,
                                    (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }


    return _task_id;
}
//change MulticopterLqrControl to multicopterFbControl
MulticopterLqrControl *MulticopterLqrControl::instantiate(int argc, char *argv[])
{				//change MulticopterLqrControl to multicopterFbControl
    return new MulticopterLqrControl();
}
//change MulticopterLqrControl to multicopterFbControl
int MulticopterLqrControl::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}
//change lqr_control_main to fb_control_main
int lqr_control_main(int argc, char *argv[])
{		//change MulticopterLqrControl to multicopterFbControl
    return MulticopterLqrControl::main(argc, argv);
}
//change MulticopterLqrControl to multicopterFbControl
MulticopterLqrControl::~MulticopterLqrControl(){
    perf_free(_loop_perf);
}
