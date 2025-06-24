#ifndef HFD_DEFINES_H
#define HFD_DEFINES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <limits.h>

typedef unsigned int  HFDuint;
typedef unsigned char HFDboolean;
typedef unsigned char HFDuchar;
typedef unsigned long HFDulong;
typedef unsigned short HFDushort;
typedef int HFDint;
typedef float HFDfloat;
typedef double HFDdouble;
typedef long HFDlong;
typedef char HFDchar;
typedef unsigned int HFDerror;
typedef unsigned int HFDenum;
typedef const char* HFDstring;
typedef unsigned int HHFD;

typedef struct  
{
	unsigned short errCode;
	unsigned char errNode;
	char* errString;
}ErrInfo;

/*Boolean*/
#define HFD_TRUE							1
#define HFD_FALSE							0

#define HFDCALLBACK  __stdcall
/******************************************************************************
* Error codes                                                                */
#define  HFD_SUCCESS						0x0000

#define  HFD_NO_DETECTED_DEVICE			0x0001

/* Communication errors */
#define  HFD_COMM_OPEN_ERROR				0x0010
#define  HFD_COMM_CONFIG_ERROR				0x0011
#define  HFD_COMM_READWRITE_ERROR			0x0012

/* Function errors */
#define  HFD_INVALID_ENUM					0x0100
#define  HFD_INVALID_RETURNVALUE			0x0101
#define  HFD_INVALID_OPERATION				0x0102
#define  HFD_INVALID_INPUT_TYPE			0x0103
#define  HFD_BAD_HANDLE					0x0104
#define  HFD_EXPERT_MODE_NOT_SET			0x0105

/* Force errors */
#define  HFD_WARM_MOTORS					0x0200
#define  HFD_EXCEEDED_MAX_FORCE			0x0201
#define  HFD_EXCEEDED_MAX_FORCE_IMPULSE	0x0202
#define  HFD_EXCEEDED_MAX_VELOCITY			0x0203
#define  HFD_FORCE_ERROR					0x0204

/* Device errors */
#define  HFD_DEVICE_NOT_OPEN				0x0300
#define  HFD_DEVICE_FAULT					0x0301
#define  HFD_DEVICE_ALREADY_INITIATED		0x0302
#define  HFD_DEVICE_NOT_INITIATED			0x0303
#define  HFD_TIMER_ERROR					0x0304
#define  HFD_MODE_ERROR					0x0305
#define  HFD_MODE_CONTROL_ERROR			0x0306
#define  HFD_REGISTER_READ_ERROR			0x0307
#define  HFD_REGISTER_WRITE_ERROR			0x0308
#define  HFD_ENCODER_READ_ERROR			0x0309
#define  HFD_MOTOR_POS_DRIVE_ERROR			0x0310
#define  HFD_MOTOR_VEL_DRIVE_ERROR			0x0311
#define  HFD_MOTOR_CURRENT_DRIVE_ERROR		0x0312
#define  HFD_DEVICE_NOT_CALIBRATED			0x0313
#define  HFD_BUTTON_STATE_READ_ERROR		0x0314
#define  HFD_LED_CONTROL_ERROR				0x0315
#define  HFD_BRAKE_CONTROL_ERROR			0x0316
#define  HFD_DEVICE_ENABLE_ERROR			0x0317

/* Haptic rendering context */
#define  HFD_ILLEGAL_BEGIN					0x0400
#define  HFD_ILLEGAL_END					0x0401
#define  HFD_FRAME_ERROR					0x0402

/* Scheduler errors */
#define  HFD_INVALID_PRIORITY				0x0500
#define  HFD_SCHEDULER_FULL				0x0501

/* Licensing erros */
#define HFD_INVALID_LICENSE				0x0600

#define HFD_DEVICE_ERROR					0x0700
/* Error codes end                                                           *
*****************************************************************************/


/*****************************************************************************
* Get parameter options                                                     */
/* Raw values */
#define HFD_CURRENT_BUTTONS				0x2000      
#define HFD_CURRENT_SAFETY_SWITCH			0x2001      
#define HFD_CURRENT_INKWELL_SWITCH			0x2002      
#define HFD_CURRENT_ENCODER_VALUES			0x2010      
#define HFD_CURRENT_PINCH_VALUE			0x2011      

/* Cartesian space values */
#define HFD_CURRENT_POSITION				0x2050      
#define HFD_CURRENT_VELOCITY				0x2051      
#define HFD_CURRENT_TRANSFORM				0x2052      
#define HFD_CURRENT_ANGULAR_VELOCITY		0x2053      
#define HFD_CURRENT_JACOBIAN				0x2054

/* Joint space values */
#define HFD_CURRENT_JOINT_ANGLES			0x2100      
#define HFD_CURRENT_GIMBAL_ANGLES			0x2150 

/* Raw values of last frame */
#define HFD_LAST_BUTTONS					0x2200      
#define HFD_LAST_SAFETY_SWITCH				0x2201      
#define HFD_LAST_INKWELL_SWITCH			0x2202      
#define HFD_LAST_ENCODER_VALUES			0x2210 
#define HFD_LAST_PINCH_VALUE				0x2012

/* Cartesian space values of last frame */
#define HFD_LAST_POSITION					0x2250      
#define HFD_LAST_VELOCITY					0x2251      
#define HFD_LAST_TRANSFORM					0x2252      
#define HFD_LAST_ANGULAR_VELOCITY			0x2253      
#define HFD_LAST_JACOBIAN					0x2254    

/* Joint space values of last frame */
#define HFD_LAST_JOINT_ANGLES				0x2300      
#define HFD_LAST_GIMBAL_ANGLES				0x2350 

/* Identification */
#define HFD_VERSION						0x2500      
#define HFD_DEVICE_MODEL_TYPE				0x2501      
#define HFD_DEVICE_DRIVER_VERSION			0x2502      
#define HFD_DEVICE_VENDOR					0x2503      
#define HFD_DEVICE_SERIAL_NUMBER			0x2504      
#define HFD_DEVICE_FIRMWARE_VERSION		0x2505

/* Device hardware properties */
#define HFD_MAX_WORKSPACE_DIMENSIONS		0x2550      
#define HFD_USABLE_WORKSPACE_DIMENSIONS	0x2551      
#define HFD_TABLETOP_OFFSET				0x2552      
#define HFD_INPUT_DOF						0x2553      
#define HFD_OUTPUT_DOF						0x2554      
#define HFD_CALIBRATION_STYLE				0x2555

/* Device forces and measurements. */
#define HFD_UPDATE_RATE                     0x2600      
#define HFD_INSTANTANEOUS_UPDATE_RATE       0x2601      
#define HFD_NOMINAL_MAX_STIFFNESS           0x2602      
#define HFD_NOMINAL_MAX_DAMPING             0x2609      
#define HFD_NOMINAL_MAX_FORCE               0x2603      
#define HFD_NOMINAL_MAX_CONTINUOUS_FORCE    0x2604      
#define HFD_MOTOR_TEMPERATURE               0x2605      
#define HFD_SOFTWARE_VELOCITY_LIMIT         0x2606      
#define HFD_SOFTWARE_FORCE_IMPULSE_LIMIT    0x2607      
#define HFD_FORCE_RAMPING_RATE              0x2608      
#define HFD_NOMINAL_MAX_TORQUE_STIFFNESS    0x2620      
#define HFD_NOMINAL_MAX_TORQUE_DAMPING      0x2621      
#define HFD_NOMINAL_MAX_TORQUE_FORCE        0x2622      
#define HFD_NOMINAL_MAX_TORQUE_CONTINUOUS_FORCE      0x2623 

/* Force and Torque */
#define HFD_CURRENT_FORCE                   0x2700      
#define HFD_CURRENT_TORQUE                  0x2701   
#define HFD_CURRENT_JOINT_TORQUE			 0x2702
#define HFD_CURRENT_GIMBAL_TORQUE			 0x2703

#define HFD_LAST_FORCE                      0x2800      
#define HFD_LAST_TORQUE                     0x2801  
#define HFD_LAST_JOINT_TORQUE               0x2802      
#define HFD_LAST_GIMBAL_TORQUE              0x2803 
/* Set parameter option end                                                  *
*****************************************************************************/

/************ EEPROM ACCESS **********/
#define HFD_DEVICE_TYPE			0x01
#define	HFD_ENCODER_RESOLUTION		0x02
#define HFD_NOMINAL_TORQUE		    0x03
#define HFD_NOMINAL_CURRENT		0x04
#define HFD_SPR_INIT_LEN			0x05
#define HFD_SPR_INIT_FORCE			0x06
#define HFD_MOT_GRAV_RTO			0x07


/*****************************************************************************
* Enable/Disable capabilities                                               */
#define HFD_FORCE_OUTPUT                    0x4000       
#define HFD_MAX_FORCE_CLAMPING              0x4001      
#define HFD_FORCE_RAMPING                   0x4002      
#define HFD_SOFTWARE_FORCE_LIMIT            0x4003 
#define HFD_ONE_FRAME_LIMIT                 0x4004 
/* Enable/Disable capabilities end                                           *
*****************************************************************************/
/*****************************************************************************


/************************************************************************/
/* CONSTANTS                                                            */
/************************************************************************/

/*devices*/
#define HFD_DEVICE_NONE				0
#define HFD_DEVICE_BASE				1
#define HFD_DEVICE_MINI				2
#define HFD_DEVICE_OMEGA3				33
#define HFD_DEVICE_OMEGA33				34
#define HFD_DEVICE_OMEGA33_LEFT		36
#define HFD_DEVICE_OMEGA331			35
#define HFD_DEVICE_OMEGA331_LEFT		37
#define HFD_DEVICE_FALCON				60
#define HFD_DEVICE_CONTROLLER			81
#define HFD_DEVICE_CONTROLLER_HR		82
#define	HFD_DEVICE_CUSTOM				91
#define HFD_DEVICE_SIGMA331			104
#define HFD_DEVICE_SIGMA331_LEFT		105
#define HFD_DEVICE_SIGMA33P			106
#define HFD_DEVICE_SIGMA33P_LEFT		107

/*status*/
#define HFD_ON							1
#define HFD_OFF						0
#define HFD_UNDEFINED					-1

#define HFD_FIRSTJOINT					0x01
#define HFD_SECONDJOINT				0x02
#define HFD_THIRDJOINT					0x04
#define HFD_FOURTHJOINT				0x08
#define HFD_FIFTHJOINT					0x10
#define HFD_SIXTHJOINT					0x20
#define HFD_SEVENTHJOINT				0x40

#define HFD_DELTAJOINT					0x07
#define HFD_WRISTJOINT					0x38
#define HFD_GRIPERJOINT				0x40

#define HFD_3DOF_DEVICE				0x07
#define HFD_6DOF_DEVICE				0x3F
#define HFD_7DOF_DEVICE				0x7F

/* degrees of freedom */
#define HFD_MAX_DOF					6

/* device count */
#define HFD_MAX_DEVICE					10

/* number of motor */
#define HFD_MOTOR_NUM					3

/* DOF of delta structure */
#define HFD_DELTA_DOF					3

/* DOF of rotation */
#define HFD_ROT_DOF					3

/* encoder count */
#define HFD_ENCODER_COUNT				6

/* delta motor index */
#define HFD_DELTA_MOTOR_0				0
#define HFD_DELTA_MOTOR_1				1
#define HFD_DELTA_MOTOR_2				2

/* wrist motor index */
#define HFD_WRIST_MOTOR_0				3
#define HFD_WRIST_MOTOR_1				4
#define HFD_WRIST_MOTOR_2				5

/* gripper motor index */
#define HFD_GRIP_MOT					6

/* delta encoder index */
#define HFD_DELTA_SHAFT_ENC_0			0
#define HFD_DELTA_SHAFT_ENC_1			1
#define HFD_DELTA_SHAFT_ENC_2			2

/* wrist encoder index */
#define HFD_WRIST_ENC_0				3
#define HFD_WRIST_ENC_1				4
#define HFD_WRIST_ENC_2				5

/* gripper encoder index */
#define HFD_GRIP_ENC					6

/* encoder extreme position */
#define HFD_MAX_POS					0x10
#define HFD_MIN_POS					0x11

/* operation mode */
#define HFD_PROFILE_POSITION_MODE		0x20
#define HFD_PROFILE_VELOCITY_MODE		0x21
#define HFD_PROFILE_TORQUE_MODE		0x22
#define HFD_INTERPOLATED_POSITION_MODE	0x23
#define HFD_RESERVED_MODE				0x25

#define HFD_CANOPEN_PROFILE			0x30
#define HFD_CUSTOM_READ_PROFILE		0x31
#define HFD_CUSTOM_WRITE_PROFILE		0x32
#define HFD_CUSTOM_ENCODER_PROFILE		0x33
#define HFD_CUSTOM_MOTOR_ENCODER_PROFILE	0x34
#define HFD_CUSTOM_MOTOR_PROFILE		0x35

/* LED */
#define HFD_LED_RED					0x01
#define HFD_LED_GREEN					0x02

/* useful non-error, positive return values */
#define HFD_TIMEGUARD					1
#define HFD_MOTOR_SATURATED			2

/* status count */
#define HFD_MAX_STATUS					16

/* status codes */
//the index of the power flag in the status array.this flag indicates if the device is powered or not.
#define HFD_STATUS_POWER            0	

//the index of the connection flag in the status array.this flag indicates if the device is connected or not.
#define HFD_STATUS_CONNECTED        1	

//the index of the start flag in the status array.this flag indicates if the device controller is running.
#define HFD_STATUS_STARTED          2

//the index of the RESET flag in the status array.this flag indicates if the device is in RESET mode or not.
#define HFD_STATUS_RESET            3

//the index of the IDLE flag in the status array.this flag indicates if the device is in IDLE mode or not.
#define HFD_STATUS_IDLE             4

//the index of the FORCE flag in the status array.this flag indicates if the device is in FORCE mode or not.
#define HFD_STATUS_FORCE            5

//the index of the BRAKE flag in the status array.this flag indicates if the device is in BRAKE mode or not.
#define HFD_STATUS_BRAKE            6

//the index of the TORQUE flag in the status array.this flag indicates if the torques are active or not when the device is in FORCE mode.
#define HFD_STATUS_TORQUE           7

//the index of the WRIST_DETECTED flag in the status array.this flag indicates if the device has a wrist or not.
#define HFD_STATUS_WRIST_DETECTED   8

//the index of the error flag in the status array.this flag indicates if an error happened on the device controller.
#define HFD_STATUS_ERROR            9

//the index of the gravity flag in the status array.this flag indicates if the gravity compensation option is enabled or not.
#define HFD_STATUS_GRAVITY         10

//the index of the TimeGuard flag in the status array.this flag indicates if the TimeGuard feature is enabled or not.
#define HFD_STATUS_TIMEGUARD       11

//the index of the WRIST_INIT flag in the status array.this flag indicates if the device wrist is initialized or not.
#define HFD_STATUS_WRIST_INIT      12

//the status of the redundant encoder consistency check.for devices equipped with redundant encoders,a value of 1
//indicates that the redundancy check is successful.a value of 0 is reported otherwise, or if the device does not feature redundant encoders.
#define HFD_STATUS_REDUNDANCY      13

//the event that caused forces to be disabled on the device(the last time forces were turned off).
//Note that not all devices support all the force-disabling mechanisms listed above.
#define HFD_STATUS_FORCEOFFCAUSE   14

#define HFD_STATUS_LOCKS           15

/* buttons count */
#define HFD_MAX_BUTTONS            16

/* velocity estimator computation mode */
#define HFD_VELOCITY_WINDOWING      0
#define HFD_VELOCITY_INSTANT        2
#define HFD_VELOCITY_WINDOW		50  // [ms]

#define HFD_VELOCITY_THRESHOLD		2000

#define HFD_RESET_MODE				 0
#define HFD_IDLE_MODE				 1
#define HFD_FORCE_MODE				 2
#define HFD_BRAKE_MODE				 3

/* USB operation modes */
#define HFD_COM_MODE_SYNC           0
#define HFD_COM_MODE_ASYNC          1
#define HFD_COM_MODE_VIRTUAL        3
#define HFD_COM_MODE_NETWORK        4

/* causes for device FORCE OFF state */
#define HFD_FORCEOFF_NONE           0		//nothing has caused forces to be turned off yet
#define HFD_FORCEOFF_BUTTON         1		//the force button was pushed
#define HFD_FORCEOFF_VELOCITY       2		//the velocity threshold was reached
#define HFD_FORCEOFF_WATCHDOG       3		//the communication watchdog kicked in
#define HFD_FORCEOFF_SOFTWARE       4		//the software requested forces to be turned off,e.g.HFDEnableForce()
#define HFD_FORCEOFF_USBDISCN       5		//the USB cable was disconnected
#define HFD_FORCEOFF_DEADMAN        6		//the dead man switch was disconnected

/* thread management */
#define HFD_THREAD_PRIORITY_DEFAULT 0
#define HFD_THREAD_PRIORITY_HIGH    1
#define HFD_THREAD_PRIORITY_LOW     2


#ifdef __cplusplus
}
#endif

#endif /*HFD_DEFINES_H*/