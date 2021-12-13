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

#include <lib/mixer/MixerBase/Mixer.hpp>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
//#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
// #include <uORB/topics/sensor_bias.h>
#include <uORB/topics/sensor_correction.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>


#include <conversion/rotation.h>
#include <lib/mathlib/mathlib.h>
#include <systemlib/err.h>
#include <px4_defines.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_armed.h>
#include <time.h>
#include <uORB/SubscriptionInterval.hpp>





//#include <px4_config.h>
//#include <systemlib/circuit_breaker.h>
//#include <systemlib/param/param.h>
//#include <systemlib/perf_counter.h>
//#include <systemlib/systemlib.h>
//#include <uORB/topics/control_state.h>
//#include <uORB/topics/eq_point.h>
//#include <uORB/topics/mc_att_control_status.h>
// #include <uORB/topics/vehicle_rate_setpoint.h>
// #include <drivers/drv_hrt.h>


using namespace matrix;

extern "C" __EXPORT int gs_control_main(int argc, char *argv[]);


class MulticopterGSControl : public ModuleBase<MulticopterGSControl>, public ModuleParams
{
public:
	MulticopterGSControl();

	virtual ~MulticopterGSControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static MulticopterGSControl *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void run() override;

	/** @see ModuleBase::print_status() */
	//int print_status() override;

	/**
	 * int		start();
	 * int		land();
	 * int		setAttControl(int flag);
	 * int		setRecControl(int flag);
	 * int		setNewEqPoint(float x, float y, float z);
	 *
	 */

private:

	void vehicle_attitude_poll(); // attitude updates
	void vehicle_position_poll(); // local position updates
	void vehicle_angular_velocity_poll(); // angular velocity updates

	void publish_actuator_control(); // publish the actuator controls

	int _v_att_sub{-1}; // vehicle attitude subscription
	struct vehicle_attitude_s _v_att {}; // vehicle attitude

	int _v_local_pos_sub{-1}; // vehicle local position subscription
	struct vehicle_local_position_s _v_local_pos {}; //vehicle local position

	orb_advert_t	_actuators_0_pub{nullptr}; // attitude actuator control publication
	struct actuator_controls_s _actuators {}; // actuator controls

	int _v_ang_vel_sub{-1}; //vehicle angular velocity subscription
	struct vehicle_angular_velocity_s _v_ang_vel {}; // vehicle angular velocity

	orb_id_t _actuators_id{nullptr};

	Matrix<float,12,1> _eq_point; // equilibrium point
	
	Matrix<float,4,1> _delta_x_int; // integrated error point

	Matrix<float,4,12> _Kerr; // proporional part of controller
	Matrix<float,4,12> _Kerr1;
	Matrix<float,4,12> _Kerr2;
	Matrix<float,4,12> _Kerr3;
	Matrix<float,4,12> _Kerr4;
	Matrix<float,4,12> _Kerr5;
	Matrix<float,4,12> _Kerr6;
	Matrix<float,4,12> _Kerr7;
	Matrix<float,4,12> _Kerr8;
	Matrix<float,4,12> _Kerr9;
	Matrix<float,4,12> _Kerr10;
	
	Matrix<float,4,4> _Kint; // integral part of controller
	Matrix<float,4,4> _Kint1;
	Matrix<float,4,4> _Kint2;
	Matrix<float,4,4> _Kint3;
	Matrix<float,4,4> _Kint4;
	Matrix<float,4,4> _Kint5;
	Matrix<float,4,4> _Kint6;
	Matrix<float,4,4> _Kint7;
	Matrix<float,4,4> _Kint8;
	Matrix<float,4,4> _Kint9;
	Matrix<float,4,4> _Kint10;

	Matrix<float,12,1> _x; // current state

	Matrix<float,4,1> _u_controls_norm; //current actuator controls

	float ff_thrust{0.8f*9.81f}; // feedforward compensation at the hovering position

	float _mass{0.8f};

	matrix::Vector3f _att_control;
	float	_thrust_sp{0.0f};

	// reading files methods

	Matrix<float,4,12> readMatrixKerr(const char *filename);

	
	Matrix<float,4,4> readMatrixKint(const char *filename);


	// writing data on files

	void writeStateOnFile(const char *filename, Matrix<float,12,1> _vect, hrt_abstime t);

	void writeActuatorControlsOnFile(const char *filename, Matrix<float,4,1> _cu, hrt_abstime t);

	// control computation

	void setCurrentState();

	void computeControls();

	perf_counter_t _loop_perf;

	void publish_actuator_controls();

	hrt_abstime _last_run;
	
	hrt_abstime _curr_run;

};
