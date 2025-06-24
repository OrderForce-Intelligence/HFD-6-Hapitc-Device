#ifndef CLAF_OPEN_H
#define CLAF_OPEN_H

#if (defined(WIN32) | defined(WIN64)) & !defined(WINCE7)
#define CLAFAPIENTRY _stdcall
#else
#define CLAFAPIENTRY
#endif

#include "clafDefines.h"
#include "clafVector.h"

#ifdef __cplusplus
extern "C"
{  // 防止函数名字改编，用extern"C"
#endif

    /****************************************************************************
     *  standard SDK
     ****************************************************************************/

    void CLAFAPIENTRY clafEnableSimulator(bool on);
    int CLAFAPIENTRY clafGetDeviceCount();
    int CLAFAPIENTRY clafGetAvailableCount();
    int CLAFAPIENTRY clafSetDevice(char ID);
    int CLAFAPIENTRY clafGetDeviceID();
    int CLAFAPIENTRY clafGetSerialNumber(CLAFushort* sn, char ID = -1);

    int CLAFAPIENTRY clafOpen();
    int CLAFAPIENTRY clafOpenType(int type);
    int CLAFAPIENTRY clafOpenSerial(int serial);
    int CLAFAPIENTRY clafOpenID(char ID);

    int CLAFAPIENTRY clafClose(char ID = -1);
    int CLAFAPIENTRY clafInit(char ID = -1);
    int CLAFAPIENTRY clafCalibrateDevice(char ID = -1);
    int CLAFAPIENTRY clafStop(char ID = -1);
    int CLAFAPIENTRY clafGetComMode(char ID = -1);
    int CLAFAPIENTRY clafEnableForce(CLAFuchar val, char ID = -1);
    int CLAFAPIENTRY clafGetOperationMode(CLAFuchar* a_mode, char ID = -1);
    int CLAFAPIENTRY clafEnableGripperForce(CLAFuchar val, char ID = -1);

    int CLAFAPIENTRY clafGetSystemType(char ID = -1);
    const char* CLAFAPIENTRY clafGetSystemName(char ID = -1);
    int CLAFAPIENTRY clafGetVersion(double* ver, char ID = -1);
    void CLAFAPIENTRY clafGetSDKVersion(int* major, int* minor, int* release, int* revision);
    int CLAFAPIENTRY clafGetStatus(int status[CLAF_MAX_STATUS], char ID = -1);

    int CLAFAPIENTRY clafGetDeviceAnlgeRad(double* angle, char ID = -1);
    int CLAFAPIENTRY clafGetDeviceAngleDeg(double* angle, char ID = -1);
    int CLAFAPIENTRY clafSetDeviceAngleRad(double angle, char ID = -1);
    int CLAFAPIENTRY clafSetDeviceAngleDeg(double angle, char ID = -1);

    int CLAFAPIENTRY clafGetBaseAngleXRad(double* angle, char ID = -1);
    int CLAFAPIENTRY clafGetBaseAngleXDeg(double* angle, char ID = -1);
    int CLAFAPIENTRY clafSetBaseAngleXRad(double angle, char ID = -1);
    int CLAFAPIENTRY clafSetBaseAngleXDeg(double angle, char ID = -1);
    int CLAFAPIENTRY clafGetBaseAngleZRad(double* angle, char ID = -1);
    int CLAFAPIENTRY clafGetBaseAngleZDeg(double* angle, char ID = -1);
    int CLAFAPIENTRY clafSetBaseAngleZRad(double angle, char ID = -1);
    int CLAFAPIENTRY clafSetBaseAngleZDeg(double angle, char ID = -1);

    int CLAFAPIENTRY clafGetEffectorMass(double* mass, char ID = -1);
    int CLAFAPIENTRY clafGetButton(int index, char ID = -1);
    unsigned int CLAFAPIENTRY clafGetButtonMask(char ID = -1);

    int CLAFAPIENTRY clafSetOutput(CLAFuint output, char ID = -1);

    bool CLAFAPIENTRY clafIsLeftHanded(char ID = -1);
    bool CLAFAPIENTRY clafHasBase(char ID = -1);
    bool CLAFAPIENTRY clafHasWrist(char ID = -1);
    bool CLAFAPIENTRY clafHasActiveWrist(char ID = -1);
    bool CLAFAPIENTRY clafHasGripper(char ID = -1);
    bool CLAFAPIENTRY clafHasActiveGripper(char ID = -1);

    int CLAFAPIENTRY clafReset(char ID = -1);
    int CLAFAPIENTRY clafResetWrist(char ID = -1);
    int CLAFAPIENTRY clafWaitForReset(int timeout = 0, char ID = -1);

    int CLAFAPIENTRY clafSetStandardGravity(double g = 9.81, char ID = -1);
    int CLAFAPIENTRY clafSetEffectorMass(double mass, char ID = -1);
    int CLAFAPIENTRY clafSetGravityCompensation(int val = CLAF_ON, char ID = -1);

    int CLAFAPIENTRY clafSetBrakes(int val = CLAF_ON, char ID = -1);

    int CLAFAPIENTRY clafGetPosition(double* px, double* py, double* pz, char ID = -1);

    int CLAFAPIENTRY clafGetOrientationRad(double* oa, double* ob, double* og, char ID = -1);
    int CLAFAPIENTRY clafGetOrientationDeg(double* oa, double* ob, double* og, char ID = -1);
    int CLAFAPIENTRY clafGetOrientationFrame(double matrix[3][3], char ID = -1);

    int CLAFAPIENTRY clafGetPositionAndOrientationRad(double* px, double* py, double* pz,
                                                      double* oa, double* ob, double* og,
                                                      char ID = -1);
    int CLAFAPIENTRY clafGetPositionAndOrientationDeg(double* px, double* py, double* pz,
                                                      double* oa, double* ob, double* og,
                                                      char ID = -1);
    int CLAFAPIENTRY clafGetPositionAndOrientationFrame(double* px, double* py, double* pz,
                                                        double matrix[3][3], char ID = -1);

    int CLAFAPIENTRY clafGetGripperAngleDeg(double* a, char ID = -1);
    int CLAFAPIENTRY clafGetGripperAngleRad(double* a, char ID = -1);
    int CLAFAPIENTRY clafGetGripperGap(double* g, char ID = -1);
    int CLAFAPIENTRY clafGetGripperThumbPos(double* px, double* py, double* pz, char ID = -1);
    int CLAFAPIENTRY clafGetGripperFingerPos(double* px, double* py, double* pz, char ID = -1);

    int CLAFAPIENTRY clafGetForce(double* fx, double* fy, double* fz, char ID = -1);
    int CLAFAPIENTRY clafSetForce(double fx, double fy, double fz, char ID = -1);

    int CLAFAPIENTRY clafGetForceAndTorque(double* fx, double* fy, double* fz, double* tx,
                                           double* ty, double* tz, char ID = -1);
    int CLAFAPIENTRY clafSetForceAndTorque(double fx, double fy, double fz, double tx, double ty,
                                           double tz, char ID = -1);

    int CLAFAPIENTRY clafSetForceAndGripperForce(double fx, double fy, double fz, double fg,
                                                 char ID = -1);
    int CLAFAPIENTRY clafSetForceAndTorqueAndGripperForce(double fx, double fy, double fz,
                                                          double tx, double ty, double tz,
                                                          double fg, char ID = -1);
    int CLAFAPIENTRY clafGetForceAndTorqueAndGripperForce(double* fx, double* fy, double* fz,
                                                          double* tx, double* ty, double* tz,
                                                          double* f, char ID = -1);

    double CLAFAPIENTRY clafGetComFreq(char ID = -1);

    int CLAFAPIENTRY clafConfigLinearVelocity(int ms = CLAF_VELOCITY_WINDOW,
                                              int mode = CLAF_VELOCITY_WINDOWING, char ID = -1);
    int CLAFAPIENTRY clafGetLinearVelocity(double* vx, double* vy, double* vz, char ID = -1);

    int CLAFAPIENTRY clafConfigAngularVelocity(int ms = CLAF_VELOCITY_WINDOW,
                                               int mode = CLAF_VELOCITY_WINDOWING, char ID = -1);
    int CLAFAPIENTRY clafGetAngularVelocityRad(double* wx, double* wy, double* wz, char ID = -1);
    int CLAFAPIENTRY clafGetAngularVelocityDeg(double* wx, double* wy, double* wz, char ID = -1);

    int CLAFAPIENTRY clafConfigGripperVelocity(int ms = CLAF_VELOCITY_WINDOW,
                                               int mode = CLAF_VELOCITY_WINDOWING, char ID = -1);
    int CLAFAPIENTRY clafGetGripperLinearVelocity(double* vg, char ID = -1);
    int CLAFAPIENTRY clafGetGripperAngularVelocityRad(double* wg, char ID = -1);
    int CLAFAPIENTRY clafGetGripperAngularVelocityDeg(double* wg, char ID = -1);

    int CLAFAPIENTRY clafEmulateButton(CLAFuchar val, char ID = -1);

    int CLAFAPIENTRY clafSetVibration(double freq, double amplitude, int type = 0, char ID = -1);

    int CLAFAPIENTRY clafErrorGetLast(unsigned short* errcode, unsigned char* errNode);
    const char* CLAFAPIENTRY clafErrorGetLastStr();
    const char* CLAFAPIENTRY clafErrorGetStr(int error);
    void CLAFAPIENTRY clafClearError();

    /****************************************************************************
     *  expert SDK
     ****************************************************************************/
    int CLAFAPIENTRY clafEnableExpertMode();
    int CLAFAPIENTRY clafDisableExpertMode();

    int CLAFAPIENTRY clafPreset(int val[CLAF_MAX_DOF], CLAFushort mask, char ID = -1);

    int CLAFAPIENTRY clafCalibrateWrist(char ID = -1);
    int CLAFAPIENTRY clafSaveCalibration(int* minValue, int* maxValue, unsigned char mask,
                                         char ID = -1);
    int CLAFAPIENTRY clafAutoCalibrate(char ID = -1);

    int CLAFAPIENTRY clafSetTimeGuard(int us, char ID = -1);

    int CLAFAPIENTRY clafSetVelocityThreshold(CLAFuint val, char ID = -1);
    int CLAFAPIENTRY clafGetVelocityThreshold(CLAFuint* val, char ID = -1);

    int CLAFAPIENTRY clafUpdateEncoders(char ID = -1);
    int CLAFAPIENTRY clafGetDeltaEncoders(int* enc0, int* enc1, int* enc2, char ID = -1);
    int CLAFAPIENTRY clafGetWristEncoders(int* enc0, int* enc1, int* enc2, char ID = -1);
    int CLAFAPIENTRY clafGetGripperEncoder(int* enc, char ID = -1);
    int CLAFAPIENTRY clafGetEncoder(int index, char ID = -1);
    int CLAFAPIENTRY clafGetEnc(int enc[CLAF_ENCODER_COUNT], CLAFuchar mask = 0x3F, char ID = -1);

    int CLAFAPIENTRY clafEnableDevice(bool a_on, char ID = -1);

    int CLAFAPIENTRY clafSetMotor(int index, short val, char ID = -1);
    int CLAFAPIENTRY clafSetDeltaMotor(short mot0, short mot1, short mot2, char ID = -1);
    int CLAFAPIENTRY clafSetWristMotor(short mot0, short mot1, short mot2, char ID = -1);
    int CLAFAPIENTRY clafSetGripperMotor(short mot, char ID = -1);

    int CLAFAPIENTRY clafDeltaEncoderToPosition(int enc0, int enc1, int enc2, double* px,
                                                double* py, double* pz, char ID = -1);
    int CLAFAPIENTRY clafDeltaPositionToEncoder(double px, double py, double pz, int* enc0,
                                                int* enc1, int* enc2, char ID = -1);
    int CLAFAPIENTRY clafDeltaEncodersToJointAngles(int enc0, int enc1, int enc2, double* j0,
                                                    double* j1, double* j2, char ID = -1);
    int CLAFAPIENTRY clafDeltaJointAnglesToEncoders(double j0, double j1, double j2, int* enc0,
                                                    int* enc1, int* enc2, char ID = -1);
    int CLAFAPIENTRY clafDeltaMotorToForce(CLAFushort mot0, CLAFushort mot1, CLAFushort mot2,
                                           int enc0, int enc1, int enc2, double* fx, double* fy,
                                           double* fz, char ID = -1);
    int CLAFAPIENTRY clafDeltaForceToMotor(double fx, double fy, double fz, int enc0, int enc1,
                                           int enc2, CLAFushort* mot0, CLAFushort* mot1,
                                           CLAFushort* mot2, char ID = -1);

    int CLAFAPIENTRY clafWristEncoderToOrientation(int enc0, int enc1, int enc2, double* oa,
                                                   double* ob, double* og, char ID = -1);
    int CLAFAPIENTRY clafWristOrientationToEncoder(double oa, double ob, double og, int* enc0,
                                                   int* enc1, int* enc2, char ID = -1);
    int CLAFAPIENTRY clafWristEncodersToJointAngles(int enc0, int enc1, int enc2, double* j0,
                                                    double* j1, double* j2, char ID = -1);
    int CLAFAPIENTRY clafWristJointAnglesToEncoders(double j0, double j1, double j2, int* enc0,
                                                    int* enc1, int* enc2, char ID = -1);
    int CLAFAPIENTRY clafWristMotorToTorque(short mot0, short mot1, short mot2, int enc0, int enc1,
                                            int enc2, double* tx, double* ty, double* tz,
                                            char ID = -1);
    int CLAFAPIENTRY clafWristTorqueToMotor(double tx, double ty, double tz, int enc0, int enc1,
                                            int enc2, short* mot0, short* mot1, short* mot2,
                                            char ID = -1);

    int CLAFAPIENTRY clafGripperEncoderToAngleRad(int enc, double* a, char ID = -1);
    int CLAFAPIENTRY clafGripperEncoderToGap(int enc, double* g, char ID = -1);
    int CLAFAPIENTRY clafGripperAngleRadToEncoder(double a, int* enc, char ID = -1);
    int CLAFAPIENTRY clafGripperGapToEncoder(double g, int* enc, char ID = -1);
    int CLAFAPIENTRY clafGripperMotorToForce(short mot, double* f, int e[4], char ID = -1);
    int CLAFAPIENTRY clafGripperForceToMotor(double f, short* mot, int e[4], char ID = -1);

    int CLAFAPIENTRY clafSetMot(short mot[CLAF_MAX_DOF], CLAFuchar mask = 0xff, char ID = -1);
    int CLAFAPIENTRY clafPreloadMot(short mot[CLAF_MAX_DOF], CLAFuchar mask = 0xff, char ID = -1);

    int CLAFAPIENTRY clafSetBrk(CLAFuchar mask = 0xff, char ID = -1);

    int CLAFAPIENTRY clafGetDeltaJointAngles(double* j0, double* j1, double* j2, char ID = -1);
    int CLAFAPIENTRY clafGetDeltaJacobian(double jcb[3][3], char ID = -1);
    int CLAFAPIENTRY clafDeltaJointAnglesToJacobian(double j0, double j1, double j2,
                                                    double jcb[3][3], char ID = -1);

    int CLAFAPIENTRY clafDeltaJointTorquesExtrema(double j0, double j1, double j2, double minq[3],
                                                  double maxq[3], char ID = -1);
    int CLAFAPIENTRY clafDeltaGravityJointTorques(double j0, double j1, double j2, double* q0,
                                                  double* q1, double* q2, char ID = -1);
    int CLAFAPIENTRY clafSetDeltaJointTorques(double t0, double t1, double t2, char ID = -1);

    int CLAFAPIENTRY clafGetWristJointAngles(double* j0, double* j1, double* j2, char ID = -1);
    int CLAFAPIENTRY clafGetWristJacobian(double jcb[3][3], char ID = -1);
    int CLAFAPIENTRY clafWristJointAnglesToJacobian(double j0, double j1, double j2,
                                                    double jcb[3][3], char ID = -1);

    int CLAFAPIENTRY clafWristJointTorquesExtrema(double j0, double j1, double j2, double minq[3],
                                                  double maxq[3], char ID = -1);
    int CLAFAPIENTRY clafWristGravityJointTorques(double j0, double j1, double j2, double* q0,
                                                  double* q1, double* q2, char ID = -1);
    int CLAFAPIENTRY clafSetWristJointTorques(double t0, double t1, double t2, char ID = -1);

    int CLAFAPIENTRY clafSetForceAndWristJointTorques(double fx, double fy, double fz, double t0,
                                                      double t1, double t2, char ID = -1);
    int CLAFAPIENTRY clafSetForceAndWristJointTorquesAndGripperForce(double fx, double fy,
                                                                     double fz, double t0,
                                                                     double t1, double t2,
                                                                     double fg, char ID = -1);

    int CLAFAPIENTRY clafGetJointTorques(double* t0, double* t1, double* t2, char ID = -1);
    int CLAFAPIENTRY clafGetJointAngles(double j[CLAF_MAX_DOF], char ID = -1);
    int CLAFAPIENTRY clafGetJointVelocities(double v[CLAF_MAX_DOF], char ID = -1);
    int CLAFAPIENTRY clafGetEncVelocities(double v[CLAF_MAX_DOF], char ID = -1);

    int CLAFAPIENTRY clafJointAnglesToInertiaMatrix(double j[CLAF_MAX_DOF], double inertia[6][6],
                                                    char ID = -1);

    int CLAFAPIENTRY clafSetComMode(int mode, char ID = -1);
    int CLAFAPIENTRY clafSetComModePriority(int priority, char ID = -1);
    int CLAFAPIENTRY clafSetWatchdog(unsigned char val, char ID = -1);
    int CLAFAPIENTRY clafGetWatchdog(unsigned char* val, char ID = -1);
    int CLAFAPIENTRY clafSetParaInt(unsigned short param, int value, char ID = -1);
    int CLAFAPIENTRY clafSetParaShort(unsigned short param, short value, char ID = -1);
    int CLAFAPIENTRY clafGetParaInt(unsigned short param, int* value, char ID = -1);
    int CLAFAPIENTRY clafGetParaShort(unsigned short param, short* value, char ID = -1);
    int CLAFAPIENTRY clafSaveConfiguration(char ID = -1);

    /****************************************************************************
     *  controller SDK
     ****************************************************************************/

    int CLAFAPIENTRY clafControllerSetDevice(int device, char ID = -1);
    int CLAFAPIENTRY clafReadConfigFromFile(char* filename, char ID = -1);

    /****************************************************************************
     *  OS independent utilities
     ****************************************************************************/

    bool CLAFAPIENTRY clafKbHit();
    char CLAFAPIENTRY clafKbGet();
    double CLAFAPIENTRY clafGetTime();
    void CLAFAPIENTRY clafSleep(double sec);
    int CLAFAPIENTRY clafStartThread(void* func(void*), void* arg, int priority);

#ifdef __cplusplus
}
#endif

#endif  // !CLAF_OPEN_H
