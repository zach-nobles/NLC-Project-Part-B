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

#include <examples/gs_control/gs_control.hpp>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_combined.h>
#include <conversion/rotation.h>
#include <drivers/drv_hrt.h>
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

using namespace matrix;
using namespace time_literals;
using namespace std;

int MulticopterGSControl::print_usage(const char *reason)
{
	if (reason){
		cout << "PX4 Warm";
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
				R"DESCR_STR(
### Description

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("gs_control","gs");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;

}

MulticopterGSControl::MulticopterGSControl():ModuleParams(nullptr),_loop_perf(perf_alloc(PC_ELAPSED,"gs_control"))
{

	for (int i=0;i<12;i++)
	{
		_eq_point(i,0) = 0.0f;
		_x(i,0) = 0.0f;
	}


	for (int i=0;i<4;i++)
	{
		_u_controls_norm(i,0) = 0.0f;
		_delta_x_int(i,0) = 0.0f;
	}
	memset(&_actuators, 0, sizeof(_actuators));


	// Build and Store the Gain Matrices for each control region (regions spaced by pi/4)
	/*
	~ regions cover -3pi/2 to pi ~
	 region 1  = -6pi/4 -> -5pi/4
	 region 2  = -5pi/4 -> -4pi/4
	 region 3  = -4pi/4 -> -3pi/4
	 region 4  = -3pi/4 -> -2pi/4
	 region 5  = -2pi/4 -> -1pi/4
	 region 6  = -1pi/4 ->  0pi/4
	 region 7  =  0pi/4 ->  1pi/4
	 region 8  =  1pi/4 ->  2pi/4
	 region 9  =  2pi/4 ->  3pi/4
	 region 10 =  3pi/4 ->  4pi/4
	*/

	// update all strings to match your local file locations
	_Kerr1 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_-6pi4.txt");
	_Kint1 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_-6pi4.txt");

	_Kerr2 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_-5pi4.txt");
	_Kint2 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_-5pi4.txt");

	_Kerr3 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_-4pi4.txt");
	_Kint3 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_-4pi4.txt");

	_Kerr4 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_-3pi4.txt");
	_Kint4 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_-3pi4.txt");

	_Kerr5 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_-2pi4.txt");
	_Kint5 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_-2pi4.txt");

	_Kerr6 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_-pi4.txt");
	_Kint6 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_-pi4.txt");

	_Kerr7 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_0.txt");
	_Kint7 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_0.txt");

	_Kerr8 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_pi4.txt");
	_Kint8 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_pi4.txt");

	_Kerr9 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_2pi4.txt");
	_Kint9 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_2pi4.txt");

	_Kerr10 = readMatrixKerr("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_err_2pi4.txt");
	_Kint10 = readMatrixKint("C:/PX4/PX4-Autopilot/src/examples/gs_control/gs_files/K_int_2pi4.txt");

}

// =================================================================================================================
// Read from File

Matrix <float,4,12> MulticopterGSControl::readMatrixKerr(const char *filename)
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

Matrix <float,4,4> MulticopterGSControl::readMatrixKint(const char *filename)
{
	static Matrix <float,4,4> result;

	static int rows = 4;
	static int cols = 4;
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

void MulticopterGSControl::setCurrentState()
{
	matrix::Quatf q;
	q = Quatf(_v_att.q);
	q.normalize();

	_x(0,0) = _v_local_pos.x;
	_x(1,0) = _v_local_pos.y;
	_x(2,0) = -_v_local_pos.z;
	_x(3,0) = _v_local_pos.vx;
	_x(4,0) = _v_local_pos.vy;
	_x(5,0) = _v_local_pos.vz;

	_x(6,0) = Eulerf(q).phi();
	_x(7,0) = Eulerf(q).theta();
	_x(8,0) = Eulerf(q).psi();
	_x(9,0) = _v_ang_vel.xyz[0];
	_x(10,0) = _v_ang_vel.xyz[1];
	_x(11,0) = _v_ang_vel.xyz[2];

}


// =================================================================================================================
// Compute Controls

void MulticopterGSControl::computeControls()
{

	static Matrix<float,4,1> u_control;
	static Matrix<float,12,1> delta_x;
	static float dt = (_curr_run - _last_run)/1e6f;
	float PI = 3.1415f;

	//============ Generate Trajectory ===============

	static float t_offset = hrt_absolute_time() + 6.0f*1e6f; /* 6e6 gives 6 seconds for us to complete liftoff */
	float t = (_last_run - t_offset)/1e6f;

	if (t>0.0f) {

		float a = 3.0f;
		float w = 3.1415f / 20.0f;
		float eps = 1e-6f;

		// Compute x component of Leminscate curve
		float x_lem = (a*cos(w*t)) / (1.0f+pow(sin(w*t),2.0f)) - a;
		float x_lem_future = (a*cos(w*(t+0.001f))) / (1.0f+pow(sin(w*(t+0.001f)),2.0f)) - a;
		_eq_point(0,0) = x_lem;

		// Compute y component of Leminscate curve
		float y_lem = (a*sin(w*t)*cos(w*t)) / (1.0f+pow(sin(w*t),2.0f));
		float y_lem_future = (a*sin(w*(t+0.001f))*cos(w*(t+0.001f))) / (1.0f+pow(sin(w*(t+0.001f)),2.0f));
		_eq_point(1,0) = y_lem;

		// Compute: yaw compent of Leminscate curve
		float yaw = 0.0f;
		if (abs(x_lem) < eps && abs(y_lem) < eps && x_lem_future > 0.0f) {
			yaw += PI/4.0f;
		}
		if (abs(x_lem) < eps && abs(y_lem) < eps && x_lem_future < 0.0f) {
			yaw += -5.0f*PI/4.0f;
		}
		if (x_lem >= eps && abs(y_lem) < eps) {
			yaw += -PI/2.0f;
		}
		if ( (y_lem > 0.0f && x_lem > 0.0f) || (y_lem < 0.0f && x_lem < 0.0f) ){
			yaw += atan2((x_lem*(pow(a,2.0f)-pow(x_lem,2.0f)-pow(y_lem,2.0f))) , (y_lem_future*(pow(a,2.0f)+pow(x_lem,2.0f)+pow(y_lem,2.0f))));
		}
		if ( (y_lem < 0.0f && x_lem >= 0.0f) || (y_lem > 0.0f && x_lem < 0.0f) ){
			yaw += atan2((x_lem*(pow(a,2.0f)-pow(x_lem,2.0f)-pow(y_lem,2.0f))) , (y_lem_future*(pow(a,2.0f)+pow(x_lem,2.0f)+pow(y_lem,2.0f)))) - PI;
		}
		// Uncomment the following line to add tangnetial yaw reference
		//_eq_point(8,0) = yaw;

		// Uncomment the follwoing line to add varying vertical height refeence
		//_eq_point(2,0) = 2.0f + 0.5f*sin(w*t);

	}else{
		_eq_point(2,0) = 2.0f; /* Liftoff to height = 2.0m */
	}

	//================================================

	delta_x = _x - _eq_point;

	//======== Integrate Trajectory Error =============

	// integate x component error
	_delta_x_int(0,0) = _delta_x_int(0,0) + dt*delta_x(0,0);

	// integate y component error
	_delta_x_int(1,0) = _delta_x_int(1,0) + dt*delta_x(1,0);

	// integate z component error
	_delta_x_int(2,0) = _delta_x_int(2,0) + dt*delta_x(2,0);

	// integate yaw component error
	if (isnan(delta_x(8,0))) {
		_delta_x_int(3,0) = _delta_x_int(3,0);
	} else {
		_delta_x_int(3,0) = _delta_x_int(3,0) + dt*delta_x(8,0);
	}

	//================================================


	//=========== Identify Controller ================

	// Region Boundaries
	float R1 = -5.0f*PI/4.0f;
	float R2 = -4.0f*PI/4.0f;
	float R3 = -3.0f*PI/4.0f;
	float R4 = -2.0f*PI/4.0f;
	float R5 = -1.0f*PI/4.0f;
	float R6 = 0.0f;
	float R7 = 1.0f*PI/4.0f;
	float hyst = 0.1f;

	// Controller Selector
	if (_x(8,0) < R1) {
		if (_Kerr == _Kerr2 && _x(8,0) > R1-hyst) {
			_Kerr = _Kerr2;
			_Kint = _Kint2;
			//cout << "Keeping K2.\n\n";
		}
		else {
		_Kerr = _Kerr1;
		_Kint = _Kint1;
		//cout << "K1.\n\n";
		}
	}
	if (_x(8,0) >= R1 && _x(8,0) < R2) {
		if (_Kerr == _Kerr1 && _x(8,0) < R1+hyst) {
			_Kerr = _Kerr1;
			_Kint = _Kint1;
			//cout << "Keeping K1.\n\n";
		}
		else if (_Kerr == _Kerr3 && _x(8,0) > R2-hyst) {
			_Kerr = _Kerr3;
			_Kint = _Kint3;
			//cout << "Keeping K3.\n\n";
		}
		else {
		_Kerr = _Kerr2;
		_Kint = _Kint2;
		//cout << "K2.\n\n";
		}
	}
	if (_x(8,0) >= R2 && _x(8,0) < R3) {
		if (_Kerr == _Kerr2 && _x(8,0) < R2+hyst) {
			_Kerr = _Kerr2;
			_Kint = _Kint2;
			//cout << "Keeping K2.\n\n";
		}
		else if (_Kerr == _Kerr4 && _x(8,0) > R3-hyst) {
			_Kerr = _Kerr4;
			_Kint = _Kint4;
			//cout << "Keeping K4.\n\n";
		}
		else {
		_Kerr = _Kerr3;
		_Kint = _Kint3;
		//cout << "K3.\n\n";
		}
	}
	if (_x(8,0) >= R3 && _x(8,0) < R4) {
		if (_Kerr == _Kerr3 && _x(8,0) < R3+hyst) {
			_Kerr = _Kerr3;
			_Kint = _Kint3;
			//cout << "Keeping K3.\n\n";
		}
		else if (_Kerr == _Kerr5 && _x(8,0) > R4-hyst) {
			_Kerr = _Kerr5;
			_Kint = _Kint5;
			//cout << "Keeping K5.\n\n";
		}
		else {
		_Kerr = _Kerr4;
		_Kint = _Kint4;
		//cout << "K4.\n\n";
		}
	}
	if (_x(8,0) >= R4 && delta_x(8,0) < R5) {
		if (_Kerr == _Kerr4 && _x(8,0) < R4+hyst) {
			_Kerr = _Kerr4;
			_Kint = _Kint4;
			//cout << "Keeping K4.\n\n";
		}
		else if (_Kerr == _Kerr6 && _x(8,0) > R5-hyst) {
			_Kerr = _Kerr6;
			_Kint = _Kint6;
			//cout << "Keeping K6.\n\n";
		}
		else {
		_Kerr = _Kerr5;
		_Kint = _Kint5;
		//cout << "K5.\n\n";
		}
	}
	if (_x(8,0) >= R5 && _x(8,0) < R6) {
		if (_Kerr == _Kerr5 && _x(8,0) < R5+hyst) {
			_Kerr = _Kerr5;
			_Kint = _Kint5;
			//cout << "Keeping K5.\n\n";
		}
		else if (_Kerr == _Kerr7 && _x(8,0) > R6-hyst) {
			_Kerr = _Kerr7;
			_Kint = _Kint7;
			//cout << "Keeping K7.\n\n";
		}
		else {
		_Kerr = _Kerr6;
		_Kint = _Kint6;
		//cout << "K6.\n\n";
		}
	}
	if (_x(8,0) >= R6 && _x(8,0) < R7) {
		if (_Kerr == _Kerr6 && _x(8,0) < R6+hyst) {
			_Kerr = _Kerr6;
			_Kint = _Kint6;
			//cout << "Keeping K6.\n\n";
		}
		else if (_Kerr == _Kerr8 && _x(8,0) > R7-hyst) {
			_Kerr = _Kerr8;
			_Kint = _Kint8;
			//cout << "Keeping K8.\n\n";
		}
		else {
		_Kerr = _Kerr7;
		_Kint = _Kint7;
		//cout << "K7.\n\n";
		}
	}
	if (_x(8,0) >= R7) {
			if (_Kerr == _Kerr7 && _x(8,0) < R7+hyst) {
			_Kerr = _Kerr7;
			_Kint = _Kint7;
			//cout << "Keeping K7.\n\n";
		}
		else {
		_Kerr = _Kerr8;
		_Kint = _Kint8;
		//cout << "K8.\n\n";
		}
	}

	//================================================

	//========== Compute Control Input ===============

	if (_delta_x_int(3,0) > 1.0f) {
		_delta_x_int(3,0) = 1.0f;
	}
	if (_delta_x_int(3,0) < -1.0f) {
		_delta_x_int(3,0) = -1.0f;
	}

	if (_delta_x_int(2,0) > 1.0f) {
		_delta_x_int(2,0) = 1.0f;
	}
	if (_delta_x_int(2,0) < -1.0f) {
		_delta_x_int(2,0) = -1.0f;
	}

	u_control = -_Kerr*(delta_x)/80.0f - _Kint*(_delta_x_int)/80.0f;
	if (t>0.0f) {
	u_control = -_Kerr*(delta_x)/25.0f - _Kint*(_delta_x_int)/25.0f;
	}

	// with full
	_u_controls_norm(1,0) = fmin(fmax((u_control(1,0))/(4.0f), -1.0f), 1.0f);
	_u_controls_norm(2,0) = fmin(fmax((u_control(2,0))/(4.0f), -1.0f), 1.0f);
	_u_controls_norm(3,0) = fmin(fmax((u_control(3,0))/(0.05f), -1.0f), 1.0f);
	_u_controls_norm(0,0) = fmin(fmax((u_control(0,0)+ff_thrust/16.0f), 0.0f), 1.0f);

	//================================================

	//========= Log State and Controls ================

	writeStateOnFile("states.txt", _x, _curr_run);
	writeActuatorControlsOnFile("controls.txt", _u_controls_norm, _curr_run);

	//================================================

	return;
}

// =================================================================================================================
// Poll EKF States

void MulticopterGSControl::vehicle_attitude_poll()
{

	// check if there is a new message
	bool updated;
	orb_check(_v_att_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude), _v_att_sub, &_v_att);
	}
	return;
}

void MulticopterGSControl::vehicle_position_poll()
{

	// check if there is a new message
	bool updated;
	orb_check(_v_local_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _v_local_pos_sub, &_v_local_pos);
	}
	return;
}

void MulticopterGSControl::vehicle_angular_velocity_poll()
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

void MulticopterGSControl::writeStateOnFile(const char *filename, Matrix<float,12,1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);
	outfile << t/1e6f << ", "; // time
	outfile << vect(0,0) << ", ";
	outfile << vect(1,0) << ", ";
	outfile << vect(2,0) << ", ";
	outfile << vect(3,0) << ", ";
	outfile << vect(4,0) << ", ";
	outfile << vect(5,0) << ", ";
	outfile << vect(6,0) << ", ";
	outfile << vect(7,0) << ", ";
	outfile << vect(8,0) << ", ";
	outfile << vect(9,0) << ", ";
	outfile << vect(10,0) << ", ";
	outfile << vect(11,0) << ";\n";
	outfile.close();
	return;
}

void MulticopterGSControl::writeActuatorControlsOnFile(const char *filename, Matrix<float,4,1> vect, hrt_abstime t) {

	ofstream outfile;
	outfile.open(filename, std::ios::out | std::ios::app);

	outfile << t/1e6f << ", "; // time
	outfile << vect(0,0) << ", ";
	outfile << vect(1,0) << ", ";
	outfile << vect(2,0) << ", ";
	outfile << vect(3,0) << ";\n";
	outfile.close();
	return;
}


// =================================================================================================================
// Publish Actuator Controls

void
MulticopterGSControl::publish_actuator_controls()
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

void MulticopterGSControl::run()
{

	// do subscriptions
	_v_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_v_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_v_ang_vel_sub = orb_subscribe(ORB_ID(vehicle_angular_velocity));

	_last_run = hrt_absolute_time();
	cout << "gs_control RUN, ... \n";

	px4_show_tasks(); // show tasks

	while (!should_exit()) {

		perf_begin(_loop_perf);
		_actuators_id = ORB_ID(actuator_controls_0);

		const hrt_abstime now = hrt_absolute_time();
		const float dt = ((now - _last_run)/1e6f);
		if (dt>0.002f){

			_curr_run = now;
			setCurrentState();
			computeControls();

			// publish actuator

			_thrust_sp = _u_controls_norm(0,0);
			_att_control(0) = _u_controls_norm(1,0);
			_att_control(1) = _u_controls_norm(2,0);
			_att_control(2) = _u_controls_norm(3,0);

			 publish_actuator_controls();
			 _last_run = now;
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


int MulticopterGSControl::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("gs_control",
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

MulticopterGSControl *MulticopterGSControl::instantiate(int argc, char *argv[])
{
	return new MulticopterGSControl();
}

int MulticopterGSControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int gs_control_main(int argc, char *argv[])
{
	return MulticopterGSControl::main(argc, argv);
}

MulticopterGSControl::~MulticopterGSControl(){
	perf_free(_loop_perf);
}
