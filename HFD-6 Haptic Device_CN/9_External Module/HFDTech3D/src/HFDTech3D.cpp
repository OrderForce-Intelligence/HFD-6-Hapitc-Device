#ifdef _MSC_VER 
#define _HAS_STD_BYTE 0
#define NOMINMAX 
#endif
#include "HFDTech3D.h"

extern "C" {

	HFDTech3D_API_DLL int hfd3DOpen()
	{
		return hfdOpen();
	}

	HFDTech3D_API_DLL int hfd3DClose(char ID)
	{
		return hfdClose(ID);
	}

	HFDTech3D_API_DLL int hfd3DInit(char ID)
	{
		return hfdInit(ID);
	}

	HFDTech3D_API_DLL int hfd3DEnableExpertMode()
	{
		return hfdEnableExpertMode();
	}

	HFDTech3D_API_DLL int hfd3DDisableExpertMode()
	{
		return hfdDisableExpertMode();
	}

	HFDTech3D_API_DLL void hfd3DClearError()
	{
		hfdClearError();
	}

	HFDTech3D_API_DLL int hfd3DGetEncoder(int index, char ID)
	{
		return hfdGetEncoder(index, ID);
	}

	HFDTech3D_API_DLL int hfd3DCalibrateDevice(char ID)
	{
		return hfdCalibrateDevice(ID);
	}

	HFDTech3D_API_DLL int hfd3DEnableDevice(bool a_on, char ID)
	{
		return hfdEnableDevice(a_on, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DEnableForce(HFDuchar val, char ID)
	{
		return hfdEnableForce(val, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DSetForce(double fx, double fy, double fz, char ID)
	{
		return hfdSetForce(fx, fy, fz, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DGetForce(double* fx, double* fy, double* fz, char ID)
	{
		return hfdGetForce(fx, fy, fz, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetOperationMode(HFDuchar* a_mode, char ID)
	{
		return hfdGetOperationMode(a_mode, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DErrorGetLast(unsigned short* internalErrcode, unsigned char* errNode)
	{
		return hfdErrorGetLast(internalErrcode, errNode);
	}

	HFDTech3D_API_DLL int hfd3DSetGravityCompensation(int val, char ID)
	{
		return hfdSetGravityCompensation(val = HFD_ON, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DGetButton(int index, char ID)
	{
		return hfdGetButton(index, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DGetDeviceCount()
	{
		return hfdGetDeviceCount();
	}

	HFDTech3D_API_DLL int hfd3DGetAvailableCount()
	{
		return hfdGetAvailableCount();
	}

	HFDTech3D_API_DLL int hfd3DSetDevice(char ID)
	{
		return hfdSetDevice(ID);
	}

	HFDTech3D_API_DLL int hfd3DGetDeviceID()
	{
		return hfdGetDeviceID();
	}

	HFDTech3D_API_DLL int hfd3DGetSerialNumber(HFDushort* sn, char ID)
	{
		return hfdGetSerialNumber(sn, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DOpenType(int type)
	{
		return hfdOpenType(type);
	}

	HFDTech3D_API_DLL int hfd3DOpenSerial(int serial)
	{
		return hfdOpenSerial(serial);
	}

	HFDTech3D_API_DLL int hfd3DOpenID(char ID)
	{
		return hfdOpenID(ID);
	}

	HFDTech3D_API_DLL int hfd3DEnableGripperForce(HFDuchar val, char ID)
	{
		return hfdEnableGripperForce(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetSystemType(char ID)
	{
		return hfdGetSystemType(ID);
	}

	HFDTech3D_API_DLL const char* hfd3DGetSystemName(char ID)
	{
		return hfdGetSystemName(ID);
	}

	HFDTech3D_API_DLL int hfd3DGetVersion(double* ver, char ID)
	{
		return hfdGetVersion(ver, ID);
	}

	HFDTech3D_API_DLL void hfd3DGetSDKVersion(int* major, int* minor, int* release, int* revision)
	{
		hfdGetSDKVersion(major, minor, release, revision);
	}

	HFDTech3D_API_DLL int hfd3DGetStatus(int status[HFD_MAX_STATUS], char ID)
	{
		return hfdGetStatus(status, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetDeviceAnlgeRad(double* angle, char ID)
	{
		return hfdGetDeviceAnlgeRad(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetDeviceAngleDeg(double* angle, char ID)
	{
		return hfdGetDeviceAngleDeg(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetDeviceAngleRad(double angle, char ID)
	{
		return hfdSetDeviceAngleRad(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetDeviceAngleDeg(double angle, char ID)
	{
		return hfdSetDeviceAngleDeg(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetBaseAngleXRad(double* angle, char ID)
	{
		return hfdGetBaseAngleXRad(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetBaseAngleXDeg(double* angle, char ID)
	{
		return hfdGetBaseAngleXDeg(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetBaseAngleXRad(double angle, char ID)
	{
		return hfdSetBaseAngleXRad(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetBaseAngleXDeg(double angle, char ID)
	{
		return hfdSetBaseAngleXDeg(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetBaseAngleZRad(double* angle, char ID)
	{
		return hfdGetBaseAngleZRad(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetBaseAngleZDeg(double* angle, char ID)
	{
		return hfdGetBaseAngleZDeg(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetBaseAngleZRad(double angle, char ID)
	{
		return hfdSetBaseAngleZRad(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetBaseAngleZDeg(double angle, char ID)
	{
		return hfdSetBaseAngleZDeg(angle, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetEffectorMass(double* mass, char ID)
	{
		return hfdGetEffectorMass(mass, ID);
	}

	HFDTech3D_API_DLL HFDuint hfd3DGetButtonMask(char ID)
	{
		return hfdGetButtonMask(ID);
	}

	HFDTech3D_API_DLL int hfd3DSetOutput(HFDuint output, char ID)
	{
		return hfdSetOutput(output, ID);
	}

	HFDTech3D_API_DLL bool hfd3DIsLeftHanded(char ID)
	{
		return hfdIsLeftHanded(ID);
	}

	HFDTech3D_API_DLL bool hfd3DHasBase(char ID)
	{
		return hfdHasBase(ID);
	}

	HFDTech3D_API_DLL bool hfd3DHasWrist(char ID)
	{
		return hfdHasWrist(ID);
	}

	HFDTech3D_API_DLL bool hfd3DHasActiveWrist(char ID)
	{
		return hfdHasActiveWrist(ID);
	}

	HFDTech3D_API_DLL bool hfd3DHasGripper(char ID)
	{
		return hfdHasGripper(ID);
	}

	HFDTech3D_API_DLL bool hfd3DHasActiveGripper(char ID)
	{
		return hfdHasActiveGripper(ID);
	}

	HFDTech3D_API_DLL int hfd3DReset(char ID)
	{
		return hfdReset(ID);
	}

	HFDTech3D_API_DLL int hfd3DResetWrist(char ID)
	{
		return hfdResetWrist(ID);
	}

	HFDTech3D_API_DLL int hfd3DWaitForReset(int timeout, char ID)
	{
		return hfdWaitForReset(timeout, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetStandardGravity(double g, char ID)
	{
		return hfdSetStandardGravity(g, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetEffectorMass(double mass, char ID)
	{
		return hfdSetEffectorMass(mass, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetBrakes(int val, char ID)
	{
		return hfdSetBrakes(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetPosition(double* px, double* py, double* pz, char ID)
	{
		return hfdGetPosition(px, py, pz, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetOrientationRad(double* oa, double* ob, double* og, char ID)
	{
		return hfdGetOrientationRad(oa, ob, og, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetOrientationDeg(double* oa, double* ob, double* og, char ID)
	{
		return hfdGetOrientationDeg(oa, ob, og, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetOrientationFrame(double matrix[3][3], char ID)
	{
		return hfdGetOrientationFrame(matrix, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetPositionAndOrientationRad(double* px, double* py, double* pz, double* oa, double* ob, double* og, char ID)
	{
		return hfdGetPositionAndOrientationRad(px, py, pz, oa, ob, og, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetPositionAndOrientationDeg(double* px, double* py, double* pz, double* oa, double* ob, double* og, char ID)
	{
		return hfdGetPositionAndOrientationDeg(px, py, pz, oa, ob, og, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetPositionAndOrientationFrame(double* px, double* py, double* pz, double matrix[3][3], char ID)
	{
		return hfdGetPositionAndOrientationFrame(px, py, pz, matrix, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperAngleDeg(double* a, char ID)
	{
		return hfdGetGripperAngleDeg(a, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperAngleRad(double* a, char ID)
	{
		return hfdGetGripperAngleRad(a, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperGap(double* g, char ID)
	{
		return hfdGetGripperGap(g, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperThumbPos(double* px, double* py, double* pz, char ID)
	{
		return hfdGetGripperThumbPos(px, py, pz, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperFingerPos(double* px, double* py, double* pz, char ID)
	{
		return hfdGetGripperFingerPos(px, py, pz, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetForceAndTorque(double* fx, double* fy, double* fz, double* tx, double* ty, double* tz, char ID)
	{
		return hfdGetForceAndTorque(fx, fy, fz, tx, ty, tz, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetForceAndTorque(double fx, double fy, double fz, double tx, double ty, double tz, char ID)
	{
		return hfdSetForceAndTorque(fx, fy, fz, tx, ty, tz, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetForceAndGripperForce(double fx, double fy, double fz, double fg, char ID)
	{
		return hfdSetForceAndGripperForce(fx, fy, fz, fg, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetForceAndTorqueAndGripperForce(double fx, double fy, double fz, double tx, double ty, double tz, double fg, char ID)
	{
		return hfdSetForceAndTorqueAndGripperForce(fx, fy, fz, tx, ty, tz, fg, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetForceAndTorqueAndGripperForce(double* fx, double* fy, double* fz, double* tx, double* ty, double* tz, double* f, char ID)
	{
		return hfdGetForceAndTorqueAndGripperForce(fx, fy, fz, tx, ty, tz, f, ID);
	}

	HFDTech3D_API_DLL double hfd3DGetComFreq(char ID)
	{
		return hfdGetComFreq(ID);
	}

	HFDTech3D_API_DLL int hfd3DConfigLinearVelocity(int ms, int mode, char ID)
	{
		return hfdConfigLinearVelocity(ms, mode, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetLinearVelocity(double* vx, double* vy, double* vz, char ID)
	{
		return hfdGetLinearVelocity(vx, vy, vz, ID);
	}

	HFDTech3D_API_DLL int hfd3DConfigAngularVelocity(int ms, int mode, char ID)
	{
		return hfdConfigAngularVelocity(ms, mode, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetAngularVelocityRad(double* wx, double* wy, double* wz, char ID)
	{
		return hfdGetAngularVelocityRad(wx, wy, wz, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetAngularVelocityDeg(double* wx, double* wy, double* wz, char ID)
	{
		return hfdGetAngularVelocityDeg(wx, wy, wz, ID);
	}

	HFDTech3D_API_DLL int hfd3DConfigGripperVelocity(int ms, int mode, char ID)
	{
		return hfdConfigGripperVelocity(ms, mode, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperLinearVelocity(double* vg, char ID)
	{
		return hfdGetGripperLinearVelocity(vg, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperAngularVelocityRad(double* wg, char ID)
	{
		return hfdGetGripperAngularVelocityRad(wg, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperAngularVelocityDeg(double* wg, char ID)
	{
		return hfdGetGripperAngularVelocityDeg(wg, ID = -1);
	}

	HFDTech3D_API_DLL int hfd3DEmulateButton(HFDuchar val, char ID)
	{
		return hfdEmulateButton(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetVibration(double freq, double amplitude, int type, char ID)
	{
		return hfdSetVibration(freq, amplitude, type, ID);
	}

	HFDTech3D_API_DLL const char* hfd3DErrorGetLastStr()
	{
		return hfdErrorGetLastStr();
	}

	HFDTech3D_API_DLL const char* hfd3DErrorGetStr(int error)
	{
		return hfdErrorGetStr(error);
	}

	HFDTech3D_API_DLL int hfd3DPreset(int val[HFD_MAX_DOF], HFDushort mask, char ID)
	{
		return hfdPreset(val, mask, ID);
	}

	HFDTech3D_API_DLL int hfd3DCalibrateWrist(char ID)
	{
		return hfdCalibrateWrist(ID);
	}

	HFDTech3D_API_DLL int hfd3DSaveCalibration(int* minValue, int* maxValue, unsigned char mask, char ID)
	{
		return hfdSaveCalibration(minValue, maxValue, mask, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetTimeGuard(int us, char ID)
	{
		return hfdSetTimeGuard(us, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetVelocityThreshold(HFDuint val, char ID)
	{
		return hfdSetVelocityThreshold(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetVelocityThreshold(HFDuint* val, char ID)
	{
		return hfdGetVelocityThreshold(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DUpdateEncoders(char ID)
	{
		return hfdUpdateEncoders(ID);
	}

	HFDTech3D_API_DLL int hfd3DGetDeltaEncoders(int* enc0, int* enc1, int* enc2, char ID)
	{
		return hfdGetDeltaEncoders(enc0, enc1, enc2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetWristEncoders(int* enc0, int* enc1, int* enc2, char ID)
	{
		return hfdGetWristEncoders(enc0, enc1, enc2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetGripperEncoder(int* enc, char ID)
	{
		return hfdGetGripperEncoder(enc, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetEnc(int enc[HFD_ENCODER_COUNT], HFDushort mask, char ID)
	{
		return hfdGetEnc(enc, mask, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetMotor(int index, HFDushort val, char ID)
	{
		return hfdSetMotor(index, val, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetDeltaMotor(HFDushort mot0, HFDushort mot1, HFDushort mot2, char ID)
	{
		return hfdSetDeltaMotor(mot0, mot1, mot2, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetWristMotor(HFDushort mot0, HFDushort mot1, HFDushort mot2, char ID)
	{
		return hfdSetWristMotor(mot0, mot1, mot2, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetGripperMotor(HFDushort mot, char ID)
	{
		return hfdSetGripperMotor(mot, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaEncoderToPosition(int enc0, int enc1, int enc2, double* px, double* py, double* pz, char ID)
	{
		return hfdDeltaEncoderToPosition(enc0, enc1, enc2, px, py, pz, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaPositionToEncoder(double px, double py, double pz, int* enc0, int* enc1, int* enc2, char ID)
	{
		return hfdDeltaPositionToEncoder(px, py, pz, enc0, enc1, enc2, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaEncodersToJointAngles(int enc0, int enc1, int enc2, double* j0, double* j1, double* j2, char ID)
	{
		return hfdDeltaEncodersToJointAngles(enc0, enc1, enc2, j0, j1, j2, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaJointAnglesToEncoders(double j0, double j1, double j2, int* enc0, int* enc1, int* enc2, char ID)
	{
		return hfdDeltaJointAnglesToEncoders(j0, j1, j2, enc0, enc1, enc2, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaMotorToForce(HFDushort mot0, HFDushort mot1, HFDushort mot2, int enc0, int enc1, int enc2, double* fx, double* fy, double* fz, char ID)
	{
		return hfdDeltaMotorToForce(mot0, mot1, mot2, enc0, enc1, enc2, fx, fy, fz, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaForceToMotor(double fx, double fy, double fz, int enc0, int enc1, int enc2, HFDushort* mot0, HFDushort* mot1, HFDushort* mot2, char ID)
	{
		return hfdDeltaForceToMotor(fx, fy, fz, enc0, enc1, enc2, mot0, mot1, mot2, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristEncoderToOrientation(int enc0, int enc1, int enc2, double* oa, double* ob, double* og, char ID)
	{
		return hfdWristEncoderToOrientation(enc0, enc1, enc2, oa, ob, og, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristOrientationToEncoder(double oa, double ob, double og, int* enc0, int* enc1, int* enc2, char ID)
	{
		return hfdWristOrientationToEncoder(oa, ob, og, enc0, enc1, enc2, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristEncodersToJointAngles(int enc0, int enc1, int enc2, double* j0, double* j1, double* j2, char ID)
	{
		return hfdWristEncodersToJointAngles(enc0, enc1, enc2, j0, j1, j2, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristJointAnglesToEncoders(double j0, double j1, double j2, int* enc0, int* enc1, int* enc2, char ID)
	{
		return hfdWristJointAnglesToEncoders(j0, j1, j2, enc0, enc1, enc2, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristMotorToTorque(HFDushort mot0, HFDushort mot1, HFDushort mot2, int enc0, int enc1, int enc2, double* tx, double* ty, double* tz, char ID)
	{
		return hfdWristMotorToTorque(mot0, mot1, mot2, enc0, enc1, enc2, tx, ty, tz, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristTorqueToMotor(double tx, double ty, double tz, int enc0, int enc1, int enc2, short* mot0, short* mot1, short* mot2, char ID)
	{
		return hfdWristTorqueToMotor(tx, ty, tz, enc0, enc1, enc2, mot0, mot1, mot2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGripperEncoderToAngleRad(int enc, double* a, char ID)
	{
		return hfdGripperEncoderToAngleRad(enc, a, ID);
	}

	HFDTech3D_API_DLL int hfd3DGripperEncoderToGap(int enc, double* g, char ID)
	{
		return hfdGripperEncoderToGap(enc, g, ID);
	}

	HFDTech3D_API_DLL int hfd3DGripperAngleRadToEncoder(double a, int* enc, char ID)
	{
		return hfdGripperAngleRadToEncoder(a, enc, ID);
	}

	HFDTech3D_API_DLL int hfd3DGripperGapToEncoder(double g, int* enc, char ID)
	{
		return hfdGripperGapToEncoder(g, enc, ID);
	}

	HFDTech3D_API_DLL int hfd3DGripperMotorToForce(HFDushort mot, double* f, int e[4], char ID)
	{
		return hfdGripperMotorToForce(mot, f, e, ID);
	}

	HFDTech3D_API_DLL int hfd3DGripperForceToMotor(double f, short* mot, int e[4], char ID)
	{
		return hfdGripperForceToMotor(f, mot, e, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetMot(short mot[HFD_MAX_DOF], HFDuchar mask, char ID)
	{
		return hfdSetMot(mot, mask, ID);
	}

	HFDTech3D_API_DLL int hfd3DPreloadMot(short mot[HFD_MAX_DOF], HFDuchar mask, char ID)
	{
		return hfdPreloadMot(mot, mask, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetBrk(HFDuchar mask, char ID)
	{
		return hfdSetBrk(mask, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetDeltaJointAngles(double* j0, double* j1, double* j2, char ID)
	{
		return hfdGetDeltaJointAngles(j0, j1, j2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetDeltaJacobian(double jcb[3][3], char ID)
	{
		return hfdGetDeltaJacobian(jcb, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID)
	{
		return hfdDeltaJointAnglesToJacobian(j0, j1, j2, jcb, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID)
	{
		return hfdDeltaJointTorquesExtrema(j0, j1, j2, minq, maxq, ID);
	}

	HFDTech3D_API_DLL int hfd3DDeltaGravityJointTorques(double j0, double j1, double j2, double* q0, double* q1, double* q2, char ID)
	{
		return hfdDeltaGravityJointTorques(j0, j1, j2, q0, q1, q2, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetDeltaJointTorques(double t0, double t1, double t2, char ID)
	{
		return hfdSetDeltaJointTorques(t0, t1, t2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetWristJointAngles(double* j0, double* j1, double* j2, char ID)
	{
		return hfdGetWristJointAngles(j0, j1, j2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetWristJacobian(double jcb[3][3], char ID)
	{
		return hfdGetWristJacobian(jcb, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristJointAnglesToJacobian(double j0, double j1, double j2, double jcb[3][3], char ID)
	{
		return hfdWristJointAnglesToJacobian(j0, j1, j2, jcb, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristJointTorquesExtrema(double j0, double j1, double j2, double minq[3], double maxq[3], char ID)
	{
		return hfdWristJointTorquesExtrema(j0, j1, j2, minq, maxq, ID);
	}

	HFDTech3D_API_DLL int hfd3DWristGravityJointTorques(double j0, double j1, double j2, double* q0, double* q1, double* q2, char ID)
	{
		return hfdWristGravityJointTorques(j0, j1, j2, q0, q1, q2, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetWristJointTorques(double t0, double t1, double t2, char ID)
	{
		return hfdSetWristJointTorques(t0, t1, t2, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetForceAndWristJointTorques(double fx, double fy, double fz, double t0, double t1, double t2, char ID)
	{
		return hfdSetForceAndWristJointTorques(fx, fy, fz, t0, t1, t2, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetForceAndWristJointTorquesAndGripperForce(double fx, double fy, double fz, double t0, double t1, double t2, double fg, char ID)
	{
		return hfdSetForceAndWristJointTorquesAndGripperForce(fx, fy, fz, t0, t1, t2, fg, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetJointTorques(double* t0, double* t1, double* t2, char ID)
	{
		return hfdGetJointTorques(t0, t1, t2, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetJointAngles(double j[HFD_MAX_DOF], char ID)
	{
		return hfdGetJointAngles(j, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetJointVelocities(double v[HFD_MAX_DOF], char ID)
	{
		return hfdGetJointVelocities(v, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetEncVelocities(double v[HFD_MAX_DOF], char ID)
	{
		return hfdGetEncVelocities(v, ID);
	}

	HFDTech3D_API_DLL int hfd3DJointAnglesToInertiaMatrix(double j[HFD_MAX_DOF], double inertia[6][6], char ID)
	{
		return hfdJointAnglesToInertiaMatrix(j, inertia, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetComMode(int mode, char ID)
	{
		return hfdSetComMode(mode, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetComModePriority(int priority, char ID)
	{
		return hfdSetComModePriority(priority, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetWatchdog(unsigned char val, char ID)
	{
		return hfdSetWatchdog(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetWatchdog(unsigned char* val, char ID)
	{
		return hfdGetWatchdog(val, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetParaInt(unsigned short param, int value, char ID)
	{
		return hfdSetParaInt(param, value, ID);
	}

	HFDTech3D_API_DLL int hfd3DSetParaShort(unsigned short param, short value, char ID)
	{
		return hfdSetParaShort(param, value, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetParaInt(unsigned short param, int* value, char ID)
	{
		return hfdGetParaInt(param, value, ID);
	}

	HFDTech3D_API_DLL int hfd3DGetParaShort(unsigned short param, short* value, char ID)
	{
		return hfdGetParaShort(param, value, ID);
	}

	HFDTech3D_API_DLL int hfd3DSaveConfiguration(char ID)
	{
		return hfdSaveConfiguration(ID);
	}

	HFDTech3D_API_DLL int hfd3DControllerSetDevice(int device, char ID)
	{
		return hfdControllerSetDevice(device, ID);
	}

	HFDTech3D_API_DLL int hfd3DReadConfigFromFile(char* filename, char ID)
	{
		return hfdReadConfigFromFile(filename, ID);
	}

	HFDTech3D_API_DLL bool hfd3DKbHit()
	{
		return hfdKbHit();
	}

	HFDTech3D_API_DLL char hfd3DKbGet()
	{
		return hfdKbGet();
	}

	HFDTech3D_API_DLL double hfd3DGetTime()
	{
		return hfdGetTime();
	}

	HFDTech3D_API_DLL void hfd3DSleep(double sec)
	{
		hfdSleep(sec);
	}

	HFDTech3D_API_DLL int hfd3DStartThread(void* func(void*), void* arg, int priority)
	{
		return hfdStartThread(func, arg, priority);
	}


}


