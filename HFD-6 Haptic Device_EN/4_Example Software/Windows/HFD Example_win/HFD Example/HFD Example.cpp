// HFD-6 Example.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//
#include<Windows.h>
#include <iostream>
#include <iomanip>
#include "HFD_OPEN.h"

using namespace std;

DWORD WINAPI updateHaptics(void* lpParam);

bool m_bOpen;
bool m_bInit;
bool m_bCalibrate;
bool m_bForceEnable;
bool m_bDeviceEnable;

bool m_bSimulationRunning;
bool m_bSimulationFinished;

double m_Pos[3] = { 0 };
double m_Orientation[3][3] = {0};
int m_fButtonPress = false;
double m_Force[3] = { 0 };

double m_LinearVel[3] = { 0 }; 
double m_OrientationRad[3] = { 0 };
double m_JointAng[6] = { 0 }; 




int main()
{
    m_bOpen = false;
    m_bInit = false;
    m_bCalibrate = false;
    m_bForceEnable = false;
    m_bDeviceEnable = false;

    m_bSimulationRunning = false;
    m_bSimulationFinished = false;

    int m_DeviceCount = hfdGetDeviceCount();
    if (!m_DeviceCount)
    {
        cout << "No HFD-6 Haptic Device Detected!" << endl;
        return 0;
    }

    int error = 0;

    error = hfdOpen();
    if (error < 0)
    {
        cout << "Open the HFD-6 Haptic Device Failed!" << endl;
        return 0;
    }
    m_bOpen = true;

    error = hfdInit();
    if (error == 0)
    {
        cout << "Init the HFD-6 Haptic Device Failed!" << endl;
        return 0;
    }
    m_bInit = true;

    error = hfdCalibrateDevice();
    if (error == 0)
    {
        cout << "Calibrate the HFD-6 Haptic Device Failed!" << endl;
        return 0;
    }
    m_bCalibrate = true;

    error = hfdEnableForce(HFD_ON);
    if (error == 0)
    {
        cout << "Failed to Enable Force Mode!" << endl;
        return 0;
    }
    m_bForceEnable = true;

    hfdEnableExpertMode();

    error = hfdEnableDevice(true);
    if (error == 0)
    {
        cout << "Failed to Enable Haptice Device!" << endl;
        return 0;
    }
    m_bDeviceEnable = true;

    error = hfdSetGravityCompensation(HFD_ON);
    if (error == 0)
    {
        cout << "Failed to Set Gravity Compensation!" << endl;
        return 0;
    }

    HANDLE hThread = CreateThread(
        NULL,
        0,
        updateHaptics,
        NULL,
        NULL,
        NULL
    );

    if (hThread)
    {
        cout << "Thread has been created successfully." << endl;
        m_bSimulationRunning = true;
    }
    else
    {
        cout << "Create Thread Failed" << endl;
        hfdClose();
        return 0;
    }

    while (m_bSimulationRunning)
    {
        system("cls");

        cout << "Position:" <<fixed<<setprecision(2)<< m_Pos[0] << "," << m_Pos[1] << "," << m_Pos[2] << endl << endl;

        cout << "Orientation:" << endl;
        cout << m_Orientation[0][0] << "," << m_Orientation[0][1] << "," << m_Orientation[0][2] << endl;
        cout << m_Orientation[1][0] << "," << m_Orientation[1][1] << "," << m_Orientation[1][2] << endl;
        cout << m_Orientation[2][0] << "," << m_Orientation[2][1] << "," << m_Orientation[2][2] << endl << endl;

        cout << "Linear Velocity:" << m_LinearVel[0] << "," << m_LinearVel[1] << "," << m_LinearVel[2]<<endl << endl;

        cout << "Orientation Rad:" << m_OrientationRad[0] << "," << m_OrientationRad[1] << "," << m_OrientationRad[2] << endl << endl;

        cout << "Joint Angles:" << m_JointAng[0] << "," << m_JointAng[1] << "," << m_JointAng[2] << "," << m_JointAng[3]
            << "," << m_JointAng[4] << "," << m_JointAng[5] << endl << endl;

        cout << "Press the Button on the end-effector to exit the application!" << endl;
    }

    hfdClose();
    
    return 0;
}


DWORD WINAPI updateHaptics(void* lpParam)
{
    int error = 0;

    while (!m_bSimulationFinished)
    {
        if (m_bOpen && m_bInit && m_bCalibrate)
        {
            hfdGetPosition(&m_Pos[0], &m_Pos[1], &m_Pos[2]);

            hfdGetOrientationFrame(m_Orientation);

            hfdGetLinearVelocity(&m_LinearVel[0], &m_LinearVel[1], &m_LinearVel[2]);

            hfdGetOrientationRad(&m_OrientationRad[0], &m_OrientationRad[1], &m_OrientationRad[2]);

            error=hfdGetJointAngles(m_JointAng);
            
            if (error == 0)
            {
                const char* errStr = hfdErrorGetLastStr();

                int result = strcmp(errStr, "Expert Mode Not Set");

                if (result==0)
                {
                    hfdEnableExpertMode();

                    hfdGetJointAngles(m_JointAng);
                }
            }

            m_fButtonPress = hfdGetButton(0);

            if (m_fButtonPress)
            {
                m_bSimulationFinished = true;
            }

            if (m_bForceEnable && m_bDeviceEnable)
            {
                hfdSetForce(m_Force[0], m_Force[1], m_Force[2]);
            }
        }
    }
    m_bSimulationRunning = false;
    return 0;
}