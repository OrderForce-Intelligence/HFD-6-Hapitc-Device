#ifndef HFD_OPEN_H
#define HFD_OPEN_H

#if(defined(WIN32)|defined(WIN64))&!defined(WINCE7)
#define  HFDAPIENTRY  _stdcall
#else
#define HFDAPIENTRY 
#endif

#include "hfdDefines.h"
#include "hfdVector.h"

#ifdef __cplusplus
extern "C" {			//防止函数名字改编，用extern"C"
#endif

/****************************************************************************
*  standard SDK
****************************************************************************/

void			HFDAPIENTRY hfdEnableSimulator(bool on);
int				HFDAPIENTRY hfdGetDeviceCount();								
int				HFDAPIENTRY hfdGetAvailableCount();							
int				HFDAPIENTRY hfdSetDevice(char ID);							
int				HFDAPIENTRY hfdGetDeviceID();									
int				HFDAPIENTRY hfdGetSerialNumber(HFDushort*sn, char ID = -1);

int				HFDAPIENTRY hfdOpen();										
int				HFDAPIENTRY hfdOpenType(int type);
int				HFDAPIENTRY hfdOpenSerial(int serial);
int				HFDAPIENTRY hfdOpenID(char ID);								

int				HFDAPIENTRY hfdClose(char ID = -1);							
int				HFDAPIENTRY hfdInit(char ID = -1);							
int				HFDAPIENTRY hfdCalibrateDevice(char ID = -1);					
int				HFDAPIENTRY hfdStop(char ID = -1);
int				HFDAPIENTRY hfdGetComMode(char ID = -1);
int				HFDAPIENTRY hfdEnableForce(HFDuchar val, char ID = -1);		
int				HFDAPIENTRY hfdGetOperationMode(HFDuchar* a_mode,char ID=-1);
int				HFDAPIENTRY hfdEnableGripperForce(HFDuchar val, char ID = -1);

int				HFDAPIENTRY hfdGetSystemType(char ID = -1);
const char*		HFDAPIENTRY hfdGetSystemName(char ID = -1);
int				HFDAPIENTRY hfdGetVersion(double *ver, char ID = -1);
void			HFDAPIENTRY hfdGetSDKVersion(int*major, int*minor, int*release, int*revision);
int				HFDAPIENTRY hfdGetStatus(int status[HFD_MAX_STATUS], char ID = -1);

int				HFDAPIENTRY hfdGetDeviceAngleRad(double *angle, char ID = -1);
int				HFDAPIENTRY hfdGetDeviceAngleDeg(double*angle, char ID = -1);
int				HFDAPIENTRY hfdSetDeviceAngleRad(double angle, char ID = -1);
int				HFDAPIENTRY hfdSetDeviceAngleDeg(double angle, char ID = -1);

int				HFDAPIENTRY hfdGetBaseAngleXRad(double *angle, char ID = -1);
int				HFDAPIENTRY hfdGetBaseAngleXDeg(double *angle, char ID = -1);
int				HFDAPIENTRY hfdSetBaseAngleXRad(double  angle, char ID = -1);
int				HFDAPIENTRY hfdSetBaseAngleXDeg(double  angle, char ID = -1);
int				HFDAPIENTRY hfdGetBaseAngleZRad(double *angle, char ID = -1);
int				HFDAPIENTRY hfdGetBaseAngleZDeg(double *angle, char ID = -1);
int				HFDAPIENTRY hfdSetBaseAngleZRad(double  angle, char ID = -1);
int				HFDAPIENTRY hfdSetBaseAngleZDeg(double  angle, char ID = -1);

int				HFDAPIENTRY hfdGetEffectorMass(double*mass, char ID = -1);
int				HFDAPIENTRY hfdGetButton(int index, char ID = -1);					
unsigned int	HFDAPIENTRY hfdGetButtonMask(char ID = -1);

int				HFDAPIENTRY hfdSetOutput(HFDuint output, char ID = -1);

bool			HFDAPIENTRY hfdIsLeftHanded(char ID = -1);
bool			HFDAPIENTRY hfdHasBase(char ID = -1);
bool			HFDAPIENTRY hfdHasWrist(char ID = -1);
bool			HFDAPIENTRY hfdHasActiveWrist(char ID = -1);
bool			HFDAPIENTRY hfdHasGripper(char ID = -1);
bool			HFDAPIENTRY hfdHasActiveGripper(char ID = -1);

int				HFDAPIENTRY hfdReset(char ID = -1);
int				HFDAPIENTRY hfdResetWrist(char ID = -1);
int				HFDAPIENTRY hfdWaitForReset(int timeout = 0, char ID = -1);

int				HFDAPIENTRY hfdSetStandardGravity(double g=9.81, char ID = -1);			
int				HFDAPIENTRY hfdSetEffectorMass(double mass, char ID = -1);
int				HFDAPIENTRY hfdSetGravityCompensation(int val = HFD_ON, char ID = -1);	

int				HFDAPIENTRY hfdSetBrakes(int val = HFD_ON, char ID = -1);				

int				HFDAPIENTRY hfdGetPosition(double*px, double*py, double*pz, char ID = -1);

int				HFDAPIENTRY hfdGetOrientationRad(double*oa, double*ob, double*og, char ID = -1);
int				HFDAPIENTRY hfdGetOrientationDeg(double*oa, double*ob, double*og, char ID = -1);
int				HFDAPIENTRY hfdGetOrientationFrame(double matrix[3][3], char ID = -1);		  

int				HFDAPIENTRY hfdGetPositionAndOrientationRad(double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
int				HFDAPIENTRY hfdGetPositionAndOrientationDeg(double *px, double *py, double *pz, double *oa, double *ob, double *og, char ID = -1);
int				HFDAPIENTRY hfdGetPositionAndOrientationFrame(double *px, double *py, double *pz, double matrix[3][3], char ID = -1);				

int				HFDAPIENTRY hfdGetGripperAngleDeg(double *a, char ID = -1);
int				HFDAPIENTRY hfdGetGripperAngleRad(double *a, char ID = -1);
int				HFDAPIENTRY hfdGetGripperGap(double *g, char ID = -1);
int				HFDAPIENTRY hfdGetGripperThumbPos(double *px, double *py, double *pz, char ID = -1);
int				HFDAPIENTRY hfdGetGripperFingerPos(double *px, double *py, double *pz, char ID = -1);

int				HFDAPIENTRY hfdGetForce(double*fx, double*fy, double*fz, char ID = -1);
int				HFDAPIENTRY hfdSetForce(double fx, double fy, double fz, char ID = -1);

int				HFDAPIENTRY hfdGetForceAndTorque(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz, char ID = -1);
int				HFDAPIENTRY hfdSetForceAndTorque(double  fx, double  fy, double  fz, double  tx, double  ty, double  tz, char ID = -1);

int				HFDAPIENTRY hfdSetForceAndGripperForce(double fx, double fy, double fz, double fg, char ID = -1);
int				HFDAPIENTRY hfdSetForceAndTorqueAndGripperForce(double fx, double fy, double fz, double tx, double ty, double tz, double fg, char ID = -1);
int				HFDAPIENTRY hfdGetForceAndTorqueAndGripperForce(double *fx, double *fy, double *fz, double *tx, double *ty, double *tz, double *f, char ID = -1);

double			HFDAPIENTRY hfdGetComFreq(char ID = -1);

int				HFDAPIENTRY hfdConfigLinearVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);
int				HFDAPIENTRY hfdGetLinearVelocity(double *vx, double *vy, double *vz, char ID = -1);

int				HFDAPIENTRY hfdConfigAngularVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);
int				HFDAPIENTRY hfdGetAngularVelocityRad(double *wx, double *wy, double *wz, char ID = -1);
int				HFDAPIENTRY hfdGetAngularVelocityDeg(double *wx, double *wy, double *wz, char ID = -1);

int				HFDAPIENTRY hfdConfigGripperVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);
int				HFDAPIENTRY hfdGetGripperLinearVelocity(double *vg, char ID = -1);
int				HFDAPIENTRY hfdGetGripperAngularVelocityRad(double *wg, char ID = -1);
int				HFDAPIENTRY hfdGetGripperAngularVelocityDeg(double *wg, char ID = -1);

int				HFDAPIENTRY hfdEmulateButton(HFDuchar val, char ID = -1);

int				HFDAPIENTRY hfdSetVibration(double freq, double amplitude, int type = 0, char ID = -1);

int				HFDAPIENTRY hfdErrorGetLast(unsigned short* errcode,unsigned char* errNode);
const char*		HFDAPIENTRY hfdErrorGetLastStr();
const char*		HFDAPIENTRY hfdErrorGetStr(int error);
void			HFDAPIENTRY hfdClearError();

/****************************************************************************
*  expert SDK
****************************************************************************/
int				HFDAPIENTRY hfdEnableExpertMode();
int				HFDAPIENTRY hfdDisableExpertMode();

int				HFDAPIENTRY hfdPreset(int val[HFD_MAX_DOF], HFDushort mask, char ID = -1);

int				HFDAPIENTRY hfdCalibrateWrist(char ID = -1);
int				HFDAPIENTRY hfdSaveCalibration(int* minValue, int* maxValue, unsigned char mask, char ID = -1);
int				HFDAPIENTRY hfdAutoCalibrate(char ID = -1);

int				HFDAPIENTRY hfdSetTimeGuard(int us, char ID = -1);

int				HFDAPIENTRY hfdSetVelocityThreshold(HFDuint val, char ID = -1); 
int				HFDAPIENTRY hfdGetVelocityThreshold(HFDuint *val, char ID = -1);

int				HFDAPIENTRY hfdUpdateEncoders(char ID = -1);								   
int				HFDAPIENTRY hfdGetDeltaEncoders(unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int				HFDAPIENTRY hfdGetWristEncoders(unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int				HFDAPIENTRY hfdGetGripperEncoder(unsigned int *enc, char ID = -1);
unsigned int	HFDAPIENTRY hfdGetEncoder(int index, char ID = -1);
int				HFDAPIENTRY hfdGetEnc(unsigned int enc[HFD_ENCODER_COUNT], HFDuchar mask = 0x3F, char ID = -1);

int				HFDAPIENTRY hfdEnableDevice(bool a_on,char ID=-1);

int				HFDAPIENTRY hfdSetMotor(int index, short val, char ID = -1);
int				HFDAPIENTRY hfdSetDeltaMotor(short mot0, short mot1, short mot2, char ID = -1);
int				HFDAPIENTRY hfdSetWristMotor(short mot0, short mot1, short mot2, char ID = -1);
int				HFDAPIENTRY hfdSetGripperMotor(short mot, char ID = -1);

int				HFDAPIENTRY hfdDeltaEncoderToPosition(unsigned int enc0, unsigned int enc1, unsigned int enc2, double *px, double *py, double *pz, char ID = -1);
int				HFDAPIENTRY hfdDeltaPositionToEncoder(double px, double py, double pz, unsigned int  *enc0, unsigned int  *enc1, unsigned int  *enc2, char ID = -1);
int				HFDAPIENTRY hfdDeltaEncodersToJointAngles(unsigned int enc0, unsigned int enc1, unsigned int enc2, double *j0, double *j1, double *j2, char ID = -1);
int				HFDAPIENTRY hfdDeltaJointAnglesToEncoders(double j0, double j1, double j2, unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int				HFDAPIENTRY hfdDeltaMotorToForce(HFDushort mot0, HFDushort mot1, HFDushort mot2, unsigned int enc0, unsigned int enc1, unsigned  int enc2, double  *fx, double  *fy, double  *fz, char ID = -1);
int				HFDAPIENTRY hfdDeltaForceToMotor(double fx, double fy, double fz, unsigned int enc0, unsigned int enc1, unsigned int enc2, HFDushort *mot0, HFDushort *mot1, HFDushort *mot2, char ID = -1);

int				HFDAPIENTRY hfdWristEncoderToOrientation(unsigned int enc0, unsigned int  enc1, unsigned int  enc2, double *oa, double *ob, double *og, char ID = -1);
int				HFDAPIENTRY hfdWristOrientationToEncoder(double oa, double ob, double og, unsigned int  *enc0, unsigned int  *enc1, unsigned int  *enc2, char ID = -1);
int				HFDAPIENTRY hfdWristEncodersToJointAngles(unsigned int enc0, unsigned int enc1, unsigned int enc2, double *j0, double *j1, double *j2, char ID = -1);
int				HFDAPIENTRY hfdWristJointAnglesToEncoders(double j0, double j1, double j2, unsigned int *enc0, unsigned int *enc1, unsigned int *enc2, char ID = -1);
int				HFDAPIENTRY hfdWristMotorToTorque(short mot0, short mot1, short mot2, unsigned int enc0, unsigned int enc1, unsigned int enc2, double *tx, double *ty, double *tz, char ID = -1);
int				HFDAPIENTRY hfdWristTorqueToMotor(double tx, double ty, double tz, unsigned int enc0, unsigned int enc1, unsigned int enc2, short *mot0, short *mot1, short *mot2, char ID = -1);

int				HFDAPIENTRY hfdGripperEncoderToAngleRad(unsigned int enc, double *a, char ID = -1);
int				HFDAPIENTRY hfdGripperEncoderToGap(unsigned int enc, double *g, char ID = -1);
int				HFDAPIENTRY hfdGripperAngleRadToEncoder(double a, unsigned int *enc, char ID = -1);
int				HFDAPIENTRY hfdGripperGapToEncoder(double g, unsigned int *enc, char ID = -1);
int				HFDAPIENTRY hfdGripperMotorToForce(short mot, double *f, unsigned int e[4], char ID = -1);
int				HFDAPIENTRY hfdGripperForceToMotor(double f, short *mot, unsigned int e[4], char ID = -1);

int				HFDAPIENTRY hfdSetMot(short mot[HFD_MAX_DOF], HFDuchar mask = 0xff, char ID = -1);
int				HFDAPIENTRY hfdPreloadMot(short mot[HFD_MAX_DOF], HFDuchar mask = 0xff, char ID = -1);

int				HFDAPIENTRY hfdSetBrk(HFDuchar mask = 0xff, char ID = -1);
	
int				HFDAPIENTRY hfdGetDeltaJointAngles(double *j0, double *j1, double *j2, char ID = -1);                                                 
int				HFDAPIENTRY hfdGetDeltaJacobian(double jcb[3][3], char ID = -1);                                                             
int				HFDAPIENTRY hfdDeltaJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID = -1);                 

int				HFDAPIENTRY hfdDeltaJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);                    
int				HFDAPIENTRY hfdDeltaGravityJointTorques(double j0, double j1, double j2, double *q0, double *q1, double *q2, char ID = -1);               
int				HFDAPIENTRY hfdSetDeltaJointTorques(double t0, double t1, double t2, char ID = -1);                                         

int				HFDAPIENTRY hfdGetWristJointAngles(double *j0, double *j1, double *j2, char ID = -1);                                               
int				HFDAPIENTRY hfdGetWristJacobian(double jcb[3][3], char ID = -1);                                                        
int				HFDAPIENTRY hfdWristJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID = -1);                       

int				HFDAPIENTRY hfdWristJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);              
int				HFDAPIENTRY hfdWristGravityJointTorques(double j0, double j1, double j2, double *q0, double *q1, double *q2, char ID = -1); 
int				HFDAPIENTRY hfdSetWristJointTorques(double t0, double t1, double t2, char ID = -1);                                                     

int				HFDAPIENTRY hfdSetForceAndWristJointTorques(double fx, double fy, double fz, double t0, double t1, double t2, char ID = -1);                     
int				HFDAPIENTRY hfdSetForceAndWristJointTorquesAndGripperForce(double fx, double fy, double fz, double t0, double t1, double t2, double fg, char ID = -1);
  
int				HFDAPIENTRY hfdGetJointTorques(double *t0, double *t1, double *t2, char ID = -1);
int				HFDAPIENTRY hfdGetJointAngles(double j[HFD_MAX_DOF], char ID = -1);                                                              
int				HFDAPIENTRY hfdGetJointVelocities(double v[HFD_MAX_DOF], char ID = -1);                                                             
int				HFDAPIENTRY hfdGetEncVelocities(double v[HFD_MAX_DOF], char ID = -1);      

int				HFDAPIENTRY hfdJointAnglesToInertiaMatrix(double j[HFD_MAX_DOF], double inertia[6][6], char ID = -1);    

int				HFDAPIENTRY hfdSetComMode(int mode, char ID = -1);                                                                           
int				HFDAPIENTRY hfdSetComModePriority(int priority, char ID = -1);                                                                       
int				HFDAPIENTRY hfdSetWatchdog(unsigned char  val, char ID = -1);                                                                  
int				HFDAPIENTRY hfdGetWatchdog(unsigned char *val, char ID = -1);  
int				HFDAPIENTRY hfdSetParaInt(unsigned short param, unsigned int value, char ID = -1);
int				HFDAPIENTRY hfdSetParaShort(unsigned short param, short value, char ID = -1);
int				HFDAPIENTRY hfdGetParaInt(unsigned short param, unsigned int* value, char ID = -1);
int				HFDAPIENTRY hfdGetParaShort(unsigned short param, short* value, char ID = -1);
int				HFDAPIENTRY hfdSaveConfiguration(char ID=-1);

/****************************************************************************
*  controller SDK
****************************************************************************/

int				HFDAPIENTRY hfdControllerSetDevice(int device, char ID = -1);
int				HFDAPIENTRY hfdReadConfigFromFile(char *filename, char ID = -1);

/****************************************************************************
*  OS independent utilities
****************************************************************************/

bool			HFDAPIENTRY hfdKbHit();
char			HFDAPIENTRY hfdKbGet();
double			HFDAPIENTRY hfdGetTime();	 
void			HFDAPIENTRY hfdSleep(double sec);
int				HFDAPIENTRY hfdStartThread(void *func(void *), void *arg, int priority); 


#ifdef __cplusplus
}
#endif

#endif // !HFD_OPEN_H
