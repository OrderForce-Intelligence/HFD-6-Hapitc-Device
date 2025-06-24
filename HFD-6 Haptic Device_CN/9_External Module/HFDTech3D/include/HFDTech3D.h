#pragma once
#ifndef hfdTech3D_H
#define hfdTech3D_H
#ifndef HFDTech3D_API_DLL
#define HFDTech3D_API_DLL __declspec(dllexport)
#endif

#include<Windows.h>
#include <stdlib.h>
#include <iostream>
#include "hfd_OPEN.h"

extern "C"
{
	//Create Device Communication
	HFDTech3D_API_DLL int hfd3DOpen();

	//Delete Device Communication
	HFDTech3D_API_DLL int hfd3DClose(char ID = -1);

	//Device Parameter Init
	HFDTech3D_API_DLL int hfd3DInit(char ID = -1);

	//Device ExpertMode Enable
	HFDTech3D_API_DLL int hfd3DEnableExpertMode();

	//Device ExpertMode Disenable
	HFDTech3D_API_DLL int hfd3DDisableExpertMode();

	//Clear Error Log
	HFDTech3D_API_DLL void hfd3DClearError();

	//Get Encoder Data
	HFDTech3D_API_DLL int hfd3DGetEncoder(int index, char ID = -1);

	//Device Auto-Calibrate 
	HFDTech3D_API_DLL int hfd3DCalibrateDevice(char ID = -1);

	//Device Enabled
	HFDTech3D_API_DLL int hfd3DEnableDevice(bool a_on, char ID /*= -1*/);//

	//Device Enabled Force
	HFDTech3D_API_DLL int hfd3DEnableForce(HFDuchar val, char ID /*=-1*/);//

	//Set Device Force  
	HFDTech3D_API_DLL int hfd3DSetForce(double fx, double fy, double fz, char ID /*=-1*/);

	//Get Device Force  
	HFDTech3D_API_DLL int	hfd3DGetForce(double* fx, double* fy, double* fz, char ID = -1);

	//Get Device Operation Mode
	HFDTech3D_API_DLL int hfd3DGetOperationMode(HFDuchar* a_mode, char ID/*=-1*/);

	//int hfd3DErrorGetLast(ref uint errcode, ref uint internalErrcode, ref uint errNode);
	//Get Device Last Error
	HFDTech3D_API_DLL int hfd3DErrorGetLast(unsigned short* internalErrcode, unsigned char* errNode);

	//Set Gravity Compensation
	HFDTech3D_API_DLL int hfd3DSetGravityCompensation(int val, char ID);

	//Get Device Button Data
	HFDTech3D_API_DLL int hfd3DGetButton(int index, char ID /*=-1*/);

	//Get Device Count
	HFDTech3D_API_DLL int	hfd3DGetDeviceCount();

	//Get Available Device Count
	HFDTech3D_API_DLL int	hfd3DGetAvailableCount();

	//Set Device Defalut Number
	HFDTech3D_API_DLL int	hfd3DSetDevice(char ID);

	//Get Device ID
	HFDTech3D_API_DLL int	hfd3DGetDeviceID();

	//Get Device Serial Number
	HFDTech3D_API_DLL int	hfd3DGetSerialNumber(HFDushort* sn, char ID = -1);

	//
	HFDTech3D_API_DLL int hfd3DOpenType(int type);

	//
	HFDTech3D_API_DLL int hfd3DOpenSerial(int serial);

	//
	HFDTech3D_API_DLL int hfd3DOpenID(char ID);

	HFDTech3D_API_DLL int	hfd3DEnableGripperForce(HFDuchar val, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetSystemType(char ID = -1);

	HFDTech3D_API_DLL const char* hfd3DGetSystemName(char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetVersion(double* ver, char ID = -1);

	HFDTech3D_API_DLL void hfd3DGetSDKVersion(int* major, int* minor, int* release, int* revision);

	HFDTech3D_API_DLL int	hfd3DGetStatus(int status[HFD_MAX_STATUS], char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetDeviceAnlgeRad(double* angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetDeviceAngleDeg(double* angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetDeviceAngleRad(double angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetDeviceAngleDeg(double angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetBaseAngleXRad(double* angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetBaseAngleXDeg(double* angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetBaseAngleXRad(double  angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetBaseAngleXDeg(double  angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetBaseAngleZRad(double* angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetBaseAngleZDeg(double* angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetBaseAngleZRad(double  angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetBaseAngleZDeg(double  angle, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetEffectorMass(double* mass, char ID = -1);

	HFDTech3D_API_DLL HFDuint hfd3DGetButtonMask(char ID = -1);

	HFDTech3D_API_DLL int	 hfd3DSetOutput(HFDuint output, char ID = -1);

	HFDTech3D_API_DLL bool hfd3DIsLeftHanded(char ID = -1);

	HFDTech3D_API_DLL bool hfd3DHasBase(char ID = -1);

	HFDTech3D_API_DLL bool hfd3DHasWrist(char ID = -1);

	HFDTech3D_API_DLL bool hfd3DHasActiveWrist(char ID = -1);

	HFDTech3D_API_DLL bool hfd3DHasGripper(char ID = -1);

	HFDTech3D_API_DLL bool hfd3DHasActiveGripper(char ID = -1);

	HFDTech3D_API_DLL int	hfd3DReset(char ID = -1);

	HFDTech3D_API_DLL int	hfd3DResetWrist(char ID = -1);

	HFDTech3D_API_DLL int	hfd3DWaitForReset(int timeout = 0, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetStandardGravity(double g = 9.81, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetEffectorMass(double mass, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetBrakes(int val = HFD_ON, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetPosition(double* px, double* py, double* pz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetOrientationRad(double* oa, double* ob, double* og, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetOrientationDeg(double* oa, double* ob, double* og, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetOrientationFrame(double matrix[3][3], char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetPositionAndOrientationRad(double* px, double* py, double* pz, double* oa, double* ob, double* og, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetPositionAndOrientationDeg(double* px, double* py, double* pz, double* oa, double* ob, double* og, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetPositionAndOrientationFrame(double* px, double* py, double* pz, double matrix[3][3], char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperAngleDeg(double* a, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperAngleRad(double* a, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperGap(double* g, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperThumbPos(double* px, double* py, double* pz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperFingerPos(double* px, double* py, double* pz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetForceAndTorque(double* fx, double* fy, double* fz, double* tx, double* ty, double* tz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetForceAndTorque(double  fx, double  fy, double  fz, double  tx, double  ty, double  tz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetForceAndGripperForce(double fx, double fy, double fz, double fg, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetForceAndTorqueAndGripperForce(double fx, double fy, double fz, double tx, double ty, double tz, double fg, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetForceAndTorqueAndGripperForce(double* fx, double* fy, double* fz, double* tx, double* ty, double* tz, double* f, char ID = -1);

	HFDTech3D_API_DLL double hfd3DGetComFreq(char ID = -1);

	HFDTech3D_API_DLL int	hfd3DConfigLinearVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetLinearVelocity(double* vx, double* vy, double* vz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DConfigAngularVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetAngularVelocityRad(double* wx, double* wy, double* wz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetAngularVelocityDeg(double* wx, double* wy, double* wz, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DConfigGripperVelocity(int ms = HFD_VELOCITY_WINDOW, int mode = HFD_VELOCITY_WINDOWING, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperLinearVelocity(double* vg, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperAngularVelocityRad(double* wg, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DGetGripperAngularVelocityDeg(double* wg, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DEmulateButton(HFDuchar val, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DSetVibration(double freq, double amplitude, int type = 0, char ID = -1);

	HFDTech3D_API_DLL const char* hfd3DErrorGetLastStr();

	HFDTech3D_API_DLL const char* hfd3DErrorGetStr(int error);

	HFDTech3D_API_DLL int hfd3DPreset(int val[HFD_MAX_DOF], HFDushort mask, char ID = -1);

	HFDTech3D_API_DLL int hfd3DCalibrateWrist(char ID = -1);

	HFDTech3D_API_DLL int hfd3DSaveCalibration(int* minValue, int* maxValue, unsigned char mask, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetTimeGuard(int us, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetVelocityThreshold(HFDuint val, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetVelocityThreshold(HFDuint* val, char ID = -1);

	HFDTech3D_API_DLL int hfd3DUpdateEncoders(char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetDeltaEncoders(int* enc0, int* enc1, int* enc2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetWristEncoders(int* enc0, int* enc1, int* enc2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetGripperEncoder(int* enc, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetEnc(int enc[HFD_ENCODER_COUNT], HFDushort mask = 0x03FF, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetMotor(int index, HFDushort val, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetDeltaMotor(HFDushort mot0, HFDushort mot1, HFDushort mot2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetWristMotor(HFDushort mot0, HFDushort mot1, HFDushort mot2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetGripperMotor(HFDushort mot, char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaEncoderToPosition(int  enc0, int  enc1, int  enc2, double* px, double* py, double* pz, char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaPositionToEncoder(double px, double py, double pz, int* enc0, int* enc1, int* enc2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaEncodersToJointAngles(int enc0, int enc1, int enc2, double* j0, double* j1, double* j2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaJointAnglesToEncoders(double j0, double j1, double j2, int* enc0, int* enc1, int* enc2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaMotorToForce(HFDushort mot0, HFDushort mot1, HFDushort mot2, int enc0, int enc1, int enc2, double* fx, double* fy, double* fz, char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaForceToMotor(double fx, double fy, double fz, int enc0, int enc1, int enc2, HFDushort* mot0, HFDushort* mot1, HFDushort* mot2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristEncoderToOrientation(int enc0, int  enc1, int  enc2, double* oa, double* ob, double* og, char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristOrientationToEncoder(double oa, double ob, double og, int* enc0, int* enc1, int* enc2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristEncodersToJointAngles(int enc0, int enc1, int enc2, double* j0, double* j1, double* j2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristJointAnglesToEncoders(double j0, double j1, double j2, int* enc0, int* enc1, int* enc2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristMotorToTorque(HFDushort mot0, HFDushort mot1, HFDushort mot2, int enc0, int enc1, int enc2, double* tx, double* ty, double* tz, char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristTorqueToMotor(double tx, double ty, double tz, int enc0, int enc1, int enc2, short* mot0, short* mot1, short* mot2, char ID);

	HFDTech3D_API_DLL int hfd3DGripperEncoderToAngleRad(int enc, double* a, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGripperEncoderToGap(int enc, double* g, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGripperAngleRadToEncoder(double a, int* enc, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGripperGapToEncoder(double g, int* enc, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGripperMotorToForce(HFDushort mot, double* f, int e[4], char ID = -1);

	HFDTech3D_API_DLL int hfd3DGripperForceToMotor(double f, short* mot, int e[4], char ID);

	HFDTech3D_API_DLL int hfd3DSetMot(short mot[HFD_MAX_DOF], HFDuchar mask, char ID);

	HFDTech3D_API_DLL int hfd3DPreloadMot(short mot[HFD_MAX_DOF], HFDuchar mask, char ID);

	HFDTech3D_API_DLL int hfd3DSetBrk(HFDuchar mask = 0xff, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetDeltaJointAngles(double* j0, double* j1, double* j2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetDeltaJacobian(double jcb[3][3], char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);

	HFDTech3D_API_DLL int hfd3DDeltaGravityJointTorques(double j0, double j1, double j2, double* q0, double* q1, double* q2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetDeltaJointTorques(double t0, double t1, double t2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetWristJointAngles(double* j0, double* j1, double* j2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetWristJacobian(double jcb[3][3], char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID = -1);

	HFDTech3D_API_DLL int hfd3DWristGravityJointTorques(double j0, double j1, double j2, double* q0, double* q1, double* q2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetWristJointTorques(double t0, double t1, double t2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetForceAndWristJointTorques(double fx, double fy, double fz, double t0, double t1, double t2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetForceAndWristJointTorquesAndGripperForce(double fx, double fy, double fz, double t0, double t1, double t2, double fg, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetJointTorques(double* t0, double* t1, double* t2, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetJointAngles(double j[HFD_MAX_DOF], char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetJointVelocities(double v[HFD_MAX_DOF], char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetEncVelocities(double v[HFD_MAX_DOF], char ID = -1);

	HFDTech3D_API_DLL int hfd3DJointAnglesToInertiaMatrix(double j[HFD_MAX_DOF], double inertia[6][6], char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetComMode(int mode, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetComModePriority(int priority, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetWatchdog(unsigned char  val, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetWatchdog(unsigned char* val, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetParaInt(unsigned short param, int value, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSetParaShort(unsigned short param, short value, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetParaInt(unsigned short param, int* value, char ID = -1);

	HFDTech3D_API_DLL int hfd3DGetParaShort(unsigned short param, short* value, char ID = -1);

	HFDTech3D_API_DLL int hfd3DSaveConfiguration(char ID = -1);

	HFDTech3D_API_DLL int	hfd3DControllerSetDevice(int device, char ID = -1);

	HFDTech3D_API_DLL int	hfd3DReadConfigFromFile(char* filename, char ID = -1);

	HFDTech3D_API_DLL bool hfd3DKbHit();

	HFDTech3D_API_DLL char hfd3DKbGet();

	HFDTech3D_API_DLL double hfd3DGetTime();

	HFDTech3D_API_DLL void hfd3DSleep(double sec);

	HFDTech3D_API_DLL int	hfd3DStartThread(void* func(void*), void* arg, int priority);
}
#endif
