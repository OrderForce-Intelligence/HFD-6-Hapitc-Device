#ifndef HFD_OPEN_H
#define HFD_OPEN_H

#include "hfdDefines.h"
#include "hfdVector.h"

#ifdef __cplusplus
extern "C" {
#endif

/****************************************************************************
*  standard SDK
****************************************************************************/

void	 hfdEnableSimulator(bool on);
int		 hfdGetDeviceCount();
int		 hfdGetAvailableCount();
int		 hfdSetDevice(char ID);
int		 hfdGetDeviceID();
int		 hfdGetSerialNumber(unsigned short*sn, char ID = -1);

int		 hfdOpen();
int		 hfdOpenType(int type);
int		 hfdOpenSerial(int serial);
int		 hfdOpenID(char ID);

int		 hfdClose(char ID = -1);
int		 hfdInit(char ID = -1);
int		 hfdCalibrateDevice(char ID = -1);
int		 hfdStop(char ID = -1);
int		 hfdGetComMode(char ID = -1);
int		 hfdEnableForce(unsigned char val, char ID = -1);
int		 hfdGetOperationMode(unsigned char* a_mode,char ID=-1);
int		 hfdEnableGripperForce(unsigned char val, char ID = -1);

int		 hfdGetSystemType(char ID = -1);
const char*	hfdGetSystemName(char ID = -1);
int		 hfdGetVersion(double *ver, char ID = -1);
void	 hfdGetSDKVersion(int*major, int*minor, int*release, int*revision);
int		 hfdGetStatus(int status[HFD_MAX_STATUS], char ID = -1);

int		 hfdGetDeviceAnlgeRad(double *angle, char ID = -1);
int		 hfdGetDeviceAngleDeg(double*angle, char ID = -1);
int		 hfdSetDeviceAngleRad(double angle, char ID = -1);
int		 hfdSetDeviceAngleDeg(double angle, char ID = -1);

int		 hfdGetBaseAngleXRad(double *angle, char ID = -1);
int		 hfdGetBaseAngleXDeg(double *angle, char ID = -1);
int      hfdSetBaseAngleXRad(double  angle, char ID = -1);
int		 hfdSetBaseAngleXDeg(double  angle, char ID = -1);
int		 hfdGetBaseAngleZRad(double *angle, char ID = -1);
int      hfdGetBaseAngleZDeg(double *angle, char ID = -1);
int		 hfdSetBaseAngleZRad(double  angle, char ID = -1);
int		 hfdSetBaseAngleZDeg(double  angle, char ID = -1);

int		 hfdGetEffectorMass(double*mass, char ID = -1);
int		 hfdGetButton(int index, char ID = -1);
unsigned int hfdGetButtonMask(char ID = -1);

int		 hfdSetOutput(unsigned int output, char ID = -1);

bool	 hfdIsLeftHanded(char ID = -1);
bool	 hfdHasBase(char ID = -1);
bool	 hfdHasWrist(char ID = -1);
bool	 hfdHasActiveWrist(char ID = -1);
bool	 hfdHasGripper(char ID = -1);
bool	 hfdHasActiveGripper(char ID = -1);

int		 hfdReset(char ID = -1);
int		 hfdResetWrist(char ID = -1);
int		 hfdWaitForReset(int timeout = 0, char ID = -1);

int		 hfdSetStandardGravity(double g=9.81, char ID = -1);
int		 hfdSetEffectorMass(double mass, char ID = -1);
int		 hfdSetGravityCompensation(int val = HFD_ON, char ID = -1);

int		 hfdSetBrakes(int val = HFD_ON, char ID = -1);

int		 hfdGetPosition(double*px, double*py, double*pz, char ID = -1);

int		hfdGetOrientationRad(double*oa, double*ob, double*og, char ID = -1);
int		hfdGetOrientationDeg(double*oa, double*ob, double*og, char ID = -1);
int		hfdGetOrientationFrame(double matrix[3][3], char ID = -1);

int		hfdGetPositionAndOrientationRad(double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
int		hfdGetPositionAndOrientationDeg(double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
int		hfdGetPositionAndOrientationFrame(double *px, double *py, double *pz, double matrix[3][3], char ID = -1);

int		hfdGetGripperAngleDeg(double *a, char ID = -1);
int		hfdGetGripperAngleRad(double *a, char ID = -1);
int		hfdGetGripperGap(double *g, char ID = -1);
int     hfdGetGripperThumbPos(double *px, double *py, double *pz, char ID = -1);
int		hfdGetGripperFingerPos(double *px, double *py, double *pz, char ID = -1);

int		hfdGetForce(double*fx, double*fy, double*fz, char ID = -1);
int		hfdSetForce(double fx, double fy, double fz, char ID = -1);

int		hfdGetForceAndTorque(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz, char ID = -1);
int		hfdSetForceAndTorque(double  fx, double  fy, double  fz, double  tx, double  ty, double  tz, char ID = -1);

int		hfdSetForceAndGripperForce(double fx, double fy, double fz, double fg, char ID = -1);
int		hfdSetForceAndTorqueAndGripperForce(double fx, double fy, double fz, double tx, double ty, double tz, double fg, char ID = -1);
int		hfdGetForceAndTorqueAndGripperForce(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz, double *f, char ID = -1);

double	hfdGetComFreq(char ID = -1);

int		hfdConfigLinearVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);
int		hfdGetLinearVelocity(double *vx, double *vy, double *vz, char ID = -1);

int		hfdConfigAngularVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);
int		hfdGetAngularVelocityRad(double *wx, double *wy, double *wz, char ID = -1);
int		hfdGetAngularVelocityDeg(double *wx, double *wy, double *wz, char ID = -1);

int		hfdConfigGripperVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);
int		hfdGetGripperLinearVelocity(double *vg, char ID = -1);
int		hfdGetGripperAngularVelocityRad(double *wg, char ID = -1);
int		hfdGetGripperAngularVelocityDeg(double *wg, char ID = -1);

int		hfdEmulateButton(unsigned char val, char ID = -1);

int		hfdSetVibration(double freq, double amplitude, int type = 0, char ID = -1);

int		hfdErrorGetLast(unsigned int* errcode,unsigned int* errNode);
const char*	hfdErrorGetLastStr();
const char*	hfdErrorGetStr(int error);
void		hfdClearError();

/****************************************************************************
*  expert SDK
****************************************************************************/
int		hfdEnableExpertMode();
int		hfdDisableExpertMode();

int		hfdPreset(int val[HFD_MAX_DOF], unsigned short mask, char ID = -1);

int		hfdCalibrateWrist(char ID = -1);
int		hfdSaveCalibration(unsigned int* minValue, unsigned int* maxValue, unsigned char mask, char ID = -1);
int		hfdAutoCalibrate(char ID = -1);

int		hfdSetTimeGuard(int us, char ID = -1);

int		hfdSetVelocityThreshold(unsigned int val, char ID = -1);
int		hfdGetVelocityThreshold(unsigned int *val, char ID = -1);

int		hfdUpdateEncoders(char ID = -1);
int		hfdGetDeltaEncoders(unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int		hfdGetWristEncoders(unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int		hfdGetGripperEncoder(unsigned int *enc, char ID = -1);
unsigned int		hfdGetEncoder(int index, char ID = -1);
int		hfdGetEnc(unsigned int enc[HFD_ENCODER_COUNT], unsigned char mask = 0x3F, char ID = -1);

int		hfdEnableDevice(bool a_on,char ID=-1);
int		hfdSetMotor(int index, short val, char ID = -1);
int		hfdSetDeltaMotor(short mot0, short mot1, short mot2, char ID = -1);
int		hfdSetWristMotor(short mot0, short mot1, short mot2, char ID = -1);
int		hfdSetGripperMotor(short mot, char ID = -1);

int		hfdDeltaEncoderToPosition(unsigned int  enc0, unsigned int  enc1, unsigned int  enc2, double *px, double *py, double *pz, char ID = -1);
int		hfdDeltaPositionToEncoder(double px, double py, double pz, unsigned int  *enc0, unsigned int  *enc1, unsigned int  *enc2, char ID = -1);
int		hfdDeltaEncodersToJointAngles(unsigned int enc0, unsigned int enc1, unsigned int enc2, double *j0, double *j1, double *j2, char ID = -1);
int		hfdDeltaJointAnglesToEncoders(double j0, double j1, double j2, unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int		hfdDeltaMotorToForce(unsigned short mot0, unsigned short mot1, unsigned short mot2, int enc0, int enc1, int enc2, double  *fx, double  *fy, double  *fz, char ID = -1);
int		hfdDeltaForceToMotor(double fx, double fy, double fz, int enc0,int enc1,int enc2, unsigned short *mot0, unsigned short *mot1, unsigned short *mot2, char ID = -1);

int		hfdWristEncoderToOrientation(unsigned int enc0, unsigned int  enc1, unsigned int  enc2, double *oa, double *ob, double *og, char ID = -1);
int		hfdWristOrientationToEncoder(double oa, double ob, double og, unsigned int  *enc0, unsigned int  *enc1, unsigned int  *enc2, char ID = -1);
int		hfdWristEncodersToJointAngles(unsigned int enc0, unsigned int enc1, unsigned int enc2, double *j0, double *j1, double *j2, char ID = -1);
int		hfdWristJointAnglesToEncoders(double j0, double j1, double j2, unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int		hfdWristMotorToTorque(short mot0, short mot1, short mot2, int enc0, int enc1, int enc2, double *tx, double *ty, double *tz, char ID = -1);
int		hfdWristTorqueToMotor(double tx, double ty, double tz, int enc0, int enc1, int enc2, short *mot0, short *mot1, short *mot2, char ID = -1);

int		hfdGripperEncoderToAngleRad(unsigned int enc, double *a, char ID = -1);
int		hfdGripperEncoderToGap(unsigned int enc, double *g, char ID = -1);
int		hfdGripperAngleRadToEncoder(double a, unsigned int *enc, char ID = -1);
int		hfdGripperGapToEncoder(double g, unsigned int *enc, char ID = -1);
int		hfdGripperMotorToForce(short mot, double *f, unsigned int e[4], char ID = -1);
int		hfdGripperForceToMotor(double f, short *mot, unsigned int e[4], char ID = -1);

int		hfdSetMot(short mot[HFD_MAX_DOF], unsigned char mask = 0xff, char ID = -1);
int		hfdPreloadMot(short mot[HFD_MAX_DOF], unsigned char mask = 0xff, char ID = -1);

int		hfdSetBrk(unsigned char mask = 0xff, char ID = -1);

int		hfdGetDeltaJointAngles(double *j0, double *j1, double *j2, char ID = -1);
int		hfdGetDeltaJacobian(double jcb[3][3], char ID = -1);
int		hfdDeltaJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID = -1);

int		hfdDeltaJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);
int		hfdDeltaGravityJointTorques(double j0, double j1, double j2, double *q0, double *q1, double *q2, char ID = -1);
int		hfdSetDeltaJointTorques(double t0, double t1, double t2, char ID = -1);

int		hfdGetWristJointAngles(double *j0, double *j1, double *j2, char ID = -1);
int		hfdGetWristJacobian(double jcb[3][3], char ID = -1);
int		hfdWristJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID = -1);

int		hfdWristJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);
int		hfdWristGravityJointTorques(double j0, double j1, double j2, double *q0, double *q1, double *q2, char ID = -1);
int		hfdSetWristJointTorques(double t0, double t1, double t2, char ID = -1);

int		hfdSetForceAndWristJointTorques(double fx, double fy, double fz, double t0, double t1, double t2, char ID = -1);
int		hfdSetForceAndWristJointTorquesAndGripperForce(double fx, double fy, double fz, double t0, double t1, double t2, double fg, char ID = -1);

int		hfdGetJointTorques(double *t0, double *t1, double *t2, char ID = -1);
int		hfdGetJointAngles(double j[HFD_MAX_DOF], char ID = -1);
int		hfdGetJointVelocities(double v[HFD_MAX_DOF], char ID = -1);
int		hfdGetEncVelocities(double v[HFD_MAX_DOF], char ID = -1);

int		hfdJointAnglesToInertiaMatrix(double j[HFD_MAX_DOF], double inertia[6][6], char ID = -1);

int		hfdSetComMode(int mode, char ID = -1);
int		hfdSetComModePriority(int priority, char ID = -1);
int		hfdSetWatchdog(unsigned char  val, char ID = -1);
int		hfdGetWatchdog(unsigned char *val, char ID = -1);
int		hfdSetParaInt(unsigned short param, unsigned int value, char ID = -1);
int		hfdSetParaShort(unsigned short param, short value, char ID = -1);
int		hfdGetParaInt(unsigned short param, unsigned int* value, char ID = -1);
int		hfdGetParaShort(unsigned short param, short* value, char ID = -1);
int		hfdSaveConfiguration(char ID=-1);

/****************************************************************************
*  controller SDK
****************************************************************************/

int		hfdControllerSetDevice(int device, char ID = -1);
int		hfdReadConfigFromFile(char *filename, char ID = -1);

/****************************************************************************
*  OS independent utilities
****************************************************************************/

bool	hfdKbHit();
char	hfdKbGet();
double	hfdGetTime();
void	hfdSleep(double sec);
int		hfdStartThread(void *func(void *), void *arg, int priority);


#ifdef __cplusplus
}
#endif


#endif // HFD_OPEN_H
