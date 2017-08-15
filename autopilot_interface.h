/**
 * @file autopilot_interface.h
 *
 * @brief Autopilot interface definition
 *
 * Functions for sending and recieving commands to an autopilot via MAVlink
 */


#ifndef AUTOPILOT_INTERFACE_H_
#define AUTOPILOT_INTERFACE_H_

// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "serial_port.h"

#include <signal.h>
#include <time.h>
#include <sys/time.h>

#include <common/mavlink.h>

// ------------------------------------------------------------------------------
//   Defines
// ------------------------------------------------------------------------------

/**
 * Defines for mavlink_set_position_target_local_ned_t.type_mask
 *
 * Bitmask to indicate which dimensions should be ignored by the vehicle
 *
 * a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of
 * the setpoint dimensions should be ignored.
 *
 * If bit 10 is set the floats afx afy afz should be interpreted as force
 * instead of acceleration.
 *
 * Mapping:
 * bit 1: x,
 * bit 2: y,
 * bit 3: z,
 * bit 4: vx,
 * bit 5: vy,
 * bit 6: vz,
 * bit 7: ax,
 * bit 8: ay,
 * bit 9: az,
 * bit 10: is force setpoint,
 * bit 11: yaw,
 * bit 12: yaw rate
 * remaining bits unused
 *
 * Combine bitmasks with bitwise &
 *
 * Example for position and yaw angle:
 * uint16_t type_mask =
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION &
 *     MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE;
 */

                                                // bit number  876543210987654321
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000110111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION_VELOCITY      0b0000110111000000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000110111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111


/**
 * Definations for mavlink_set_attitude_target_t's member of type_mask

< Mappings: If any of these bits are set, the corresponding input should be ignored: 
	bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, 
	bit 7: throttle, bit 8: attitude

 */

// #define MAVLINK_MSG_SET_ATTITUDE_TARGET_THROTTLE     0b00111000
#define MAVLINK_MSG_SET_ATTITUDE_TARGET_ATTITUDE     0b01111111



/*
 Bitmask to indicate which dimensions should be ignored by the vehicle: 
	 a value of 0b0000000000000000 or 0b0000001000000000 indicates 
	 that none of the setpoint dimensions should be ignored. 
	 If bit 10 is set the floats afx afy afz should be interpreted as `force` instead of `acceleration`. 
	 Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, 
	 bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate

typedef struct __mavlink_set_position_target_local_ned_t
{
 uint32_t time_boot_ms; ///< Timestamp in milliseconds since system boot
 float x; ///< X Position in NED frame in meters
 float y; ///< Y Position in NED frame in meters
 float z; ///< Z Position in NED frame in meters (note, altitude is negative in NED)
 float vx; ///< X velocity in NED frame in meter / s
 float vy; ///< Y velocity in NED frame in meter / s
 float vz; ///< Z velocity in NED frame in meter / s
 float afx; ///< X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afy; ///< Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float afz; ///< Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N
 float yaw; ///< yaw setpoint in rad
 float yaw_rate; ///< yaw rate setpoint in rad/s
 uint16_t type_mask; ///< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t coordinate_frame; ///< Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9
} mavlink_set_position_target_local_ned_t;


< Mappings: If any of these bits are set, the corresponding input should be ignored: 
	bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, 
	bit 7: throttle, bit 8: attitude

typedef struct __mavlink_set_attitude_target_t
{
 uint32_t time_boot_ms; ///< Timestamp in milliseconds since system boot
 float q[4]; ///< Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
 float body_roll_rate; ///< Body roll rate in radians per second
 float body_pitch_rate; ///< Body roll rate in radians per second
 float body_yaw_rate; ///< Body roll rate in radians per second
 float thrust; ///< Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)
 uint8_t target_system; ///< System ID
 uint8_t target_component; ///< Component ID
 uint8_t type_mask; ///< Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude
} mavlink_set_attitude_target_t;

*/


// ------------------------------------------------------------------------------
//   Prototypes
// ------------------------------------------------------------------------------


// helper functions
uint64_t get_time_usec();
void set_position(float x, float y, float z, mavlink_set_position_target_local_ned_t &sp);
void set_velocity(float vx, float vy, float vz, mavlink_set_position_target_local_ned_t &sp);
void set_acceleration(float ax, float ay, float az, mavlink_set_position_target_local_ned_t &sp);
void set_yaw(float yaw, mavlink_set_position_target_local_ned_t &sp);
void set_yaw_rate(float yaw_rate, mavlink_set_position_target_local_ned_t &sp);
void set_land( mavlink_set_position_target_local_ned_t &sp);

void* start_autopilot_interface_read_thread(void *args);
void* start_autopilot_interface_write_thread(void *args);


// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

struct Time_Stamps
{
	Time_Stamps()
	{
		reset_timestamps();
	}

	uint64_t heartbeat;
	uint64_t sys_status;
	uint64_t battery_status;
	uint64_t radio_status;
	uint64_t local_position_ned;
	uint64_t global_position_int;
	uint64_t position_target_local_ned;
	uint64_t position_target_global_int;
	uint64_t highres_imu;
	uint64_t attitude;
	uint64_t vfr_hud;

	void
	reset_timestamps()
	{
		heartbeat = 0;
		sys_status = 0;
		battery_status = 0;
		radio_status = 0;
		local_position_ned = 0;
		global_position_int = 0;
		position_target_local_ned = 0;
		position_target_global_int = 0;
		highres_imu = 0;
		attitude = 0;
		vfr_hud = 0;
	}

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

	int sysid;
	int compid;

	// Heartbeat
	mavlink_heartbeat_t heartbeat;

	// System Status
	mavlink_sys_status_t sys_status;

	// Battery Status
	mavlink_battery_status_t battery_status;

	// Radio Status
	mavlink_radio_status_t radio_status;

	// Local Position
	mavlink_local_position_ned_t local_position_ned;

	// Global Position
	mavlink_global_position_int_t global_position_int;

	// Local Position Target
	mavlink_position_target_local_ned_t position_target_local_ned;

	// Global Position Target
	mavlink_position_target_global_int_t position_target_global_int;

	// HiRes IMU
	mavlink_highres_imu_t highres_imu;

	// Attitude
	mavlink_attitude_t attitude;

	// VFR_HUD
	mavlink_vfr_hud_t vfr_hud;

	// System Parameters?


	// Time Stamps
	Time_Stamps time_stamps;

	void
	reset_timestamps()
	{
		time_stamps.reset_timestamps();
	}

};


// ----------------------------------------------------------------------------------
//   Autopilot Interface Class
// ----------------------------------------------------------------------------------
/*
 * Autopilot Interface Class
 *
 * This starts two threads for read and write over MAVlink. The read thread
 * listens for any MAVlink message and pushes it to the current_messages
 * attribute.  The write thread at the moment only streams a position target
 * in the local NED frame (mavlink_set_position_target_local_ned_t), which
 * is changed by using the method update_setpoint().  Sending these messages
 * are only half the requirement to get response from the autopilot, a signal
 * to enter "offboard_control" mode is sent by using the enable_offboard_control()
 * method.  Signal the exit of this mode with disable_offboard_control().  It's
 * important that one way or another this program signals offboard mode exit,
 * otherwise the vehicle will go into failsafe.
 */
class Autopilot_Interface
{

public:

	Autopilot_Interface();
	Autopilot_Interface(Serial_Port *serial_port_);
	~Autopilot_Interface();

	char reading_status;
	char writing_status;
	char control_status;
	char setpoint_send_status;
	uint64_t write_count;

    	int system_id;
	int autopilot_id;
	int companion_id;

	Mavlink_Messages current_messages;
	mavlink_set_position_target_local_ned_t initial_position;

	void update_setpoint(mavlink_set_position_target_local_ned_t setpoint);
	void read_messages();
	int  write_message(mavlink_message_t message);

	/*
		Set paramters of PX4 instead of qgroundcontrol

		@type: can be the following options:
			MAV_PARAM_TYPE_UINT8	 // 8-bit unsigned integer 
			MAV_PARAM_TYPE_INT8  	// 8-bit signed integer 
			MAV_PARAM_TYPE_UINT16	 // 16-bit unsigned integer
			MAV_PARAM_TYPE_INT16 	// 16-bit signed integer
			MAV_PARAM_TYPE_UINT32	//32-bit unsigned integer 
			MAV_PARAM_TYPE_INT32 	// 32-bit signed integer
			MAV_PARAM_TYPE_UINT64	// 64-bit unsigned integer 
			MAV_PARAM_TYPE_INT64 	// 64-bit signed integer 
			MAV_PARAM_TYPE_REAL32	// 32-bit floating-point 
			MAV_PARAM_TYPE_REAL64	// 64-bit floating-point
			MAV_PARAM_TYPE_ENUM_END
	*/
	void set_parameters(const char *name, float value, uint8_t type);


	void enable_offboard_control();
	void disable_offboard_control();
	bool is_in_offboard_mode();
	char get_setpoint_sendstatus();
	void set_setpoint_sendstatus(char status);

	int toggle_land_control( bool flag );
	int toggle_return_control( bool flag );
	void vehicle_armed();
	void vehicle_disarm();
	bool is_armed();

	void start();
	void stop();

	void start_read_thread();
	void start_write_thread(void);

	void handle_quit( int sig );

	void write_set_att();

private:

	Serial_Port *serial_port;

	bool time_to_exit;

	pthread_t read_tid;
	pthread_t write_tid;

	mavlink_set_position_target_local_ned_t current_setpoint;

	void read_thread();
	void write_thread(void);

	int toggle_offboard_control( bool flag );
	int toggle_arm_disarm( bool flag );
	void write_setpoint();

};

#endif // AUTOPILOT_INTERFACE_H_


