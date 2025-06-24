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
typedef int Handle;

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

/* Set parameter option end                                                  *
*****************************************************************************/


// ********************************************************************** */
// * CONSTANTS                                                            */
// ************************************************************************/

/*devices*/
#define HFD_DEVICE_NONE				0
#define HFD_DEVICE_BASE				1
#define HFD_DEVICE_MINI				2
#define HFD_DEVICE_OMEGA3			33
#define HFD_DEVICE_OMEGA33			34
#define HFD_DEVICE_OMEGA33_LEFT		36
#define HFD_DEVICE_OMEGA331			35
#define HFD_DEVICE_OMEGA331_LEFT	37
#define HFD_DEVICE_FALCON			60
#define HFD_DEVICE_CONTROLLER		81
#define HFD_DEVICE_CONTROLLER_HR	82
#define	HFD_DEVICE_CUSTOM			91
#define HFD_DEVICE_SIGMA331			104
#define HFD_DEVICE_SIGMA331_LEFT	105
#define HFD_DEVICE_SIGMA33P			106
#define HFD_DEVICE_SIGMA33P_LEFT	107

/* operation mode  */
#define HFD_PROFILE_POSITION_MODE       0x20
#define HFD_PROFILE_VELOCITY_MODE       0x21
#define HFD_PROFILE_TORQUE_MODE         0x22
#define HFD_INTERPOLATED_POSITION_MODE  0x23
#define HFD_RESERVED_MODE               0x25
/***************************************************/

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
#define HFD_GRIP_ENC				6

/* encoder extreme position */
#define HFD_MAX_POS					0x10
#define HFD_MIN_POS					0x11

/* LED */
#define HFD_LED_RED					0x01
#define HFD_LED_GREEN				0x02

/* useful non-error, positive return values */
#define HFD_TIMEGUARD				1
#define HFD_MOTOR_SATURATED			2

/* buttons count */
#define HFD_MAX_BUTTONS            16

/* velocity estimator computation mode */
#define HFD_VELOCITY_WINDOWING      0
#define HFD_VELOCITY_INSTANT        2
#define HFD_VELOCITY_WINDOW         50  // [ms]

#define HFD_VELOCITY_THRESHOLD		20000

/* thread management */
#define HFD_THREAD_PRIORITY_DEFAULT 0
#define HFD_THREAD_PRIORITY_HIGH    1
#define HFD_THREAD_PRIORITY_LOW     2

#define HFD_MAX_STATUS 4


#ifdef __cplusplus
}
#endif

#endif /*HFD_DEFINES_H*/
