#include <QCoreApplication>
#include <iostream>
#include <pthread.h>
#include <unistd.h>
#include "HFD_OPEN.h"
#include <QApplication>

using namespace std;

void* updateHaptics(void*);

bool m_bOpen;
bool m_bInit;
bool m_bCalibrate;
bool m_bForceEnable;
bool m_bDeviceEnable;

bool m_bSimulationRunning;
bool m_bSimulationFinished;

double m_Pos[3]={0};
double m_Orientation[3][3]={{0}};
int m_fButtonPress=false;
double m_Force[3]={0};
double m_AngVelDeg[3]={0};
double m_JointAngRad[6]={0};


int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);


    m_bOpen=false;
    m_bInit=false;
    m_bCalibrate=false;
    m_bForceEnable=false;
    m_bDeviceEnable=false;

    m_bSimulationRunning=false;
    m_bSimulationFinished=false;

    int m_DeviceCount=hfdGetDeviceCount();
    if(!m_DeviceCount)
    {
        cout<<"No HFD Haptic Device Detected!"<<endl;
        return 0;
    }

    int error=0;
    error=hfdOpen();
    if(error==0)
    {
        cout<<"Open the HFD Haptic Device Failed!"<<endl;
        return 0;
    }
    else
    {
        cout<<"Open the HFD Haptic Device Successed!"<<endl;
        m_bOpen=true;
    }

    sleep(1);


    error=hfdInit();
    if(error==0)
    {
        cout<<"Init the HFD Haptic Device Failed!"<<endl;
        return 0;
    }
    else
    {
        cout<<"Init the HFD Haptic Device Successed!"<<endl;
        m_bInit=true;
    }


    error=hfdCalibrateDevice();
    if(error==0)
    {
        cout<<"Calibrate the HFD Haptic Device Failed!"<<endl;
        return 0;
    }
    else
    {
        cout<<"Calibrate the HFD Haptic Device Successed!"<<endl;
        m_bCalibrate=true;
    }


    error=hfdEnableForce(HFD_ON);
    if(error==0)
    {
        cout<<"Failed to Enable Force Mode!"<<endl;
        return 0;
    }
    else
    {
        cout<<"Enable Force Mode Successed!"<<endl;
        m_bForceEnable=true;
    }


    hfdEnableExpertMode();

    error=hfdEnableDevice(true);
    if(error==0)
    {
        cout<<"Failed to Enable Haptic Device!"<<endl;
        return 0;
    }
    else
    {
        cout<<"Enable Haptic Device Successed!"<<endl;
        m_bDeviceEnable=true;
    }


    error=hfdSetGravityCompensation(HFD_ON);
    if(error==0)
    {
        cout<<"Failed to Set Gravity Compensation!"<<endl;
        return 0;
    }
    else
    {
         cout<<"Set Gravity Compensation Successed!"<<endl;
    }

    pthread_t pth_Haptic;
    error=pthread_create(&pth_Haptic,NULL,updateHaptics,NULL);

    if(error==0)
    {
        cout<<"Thread has been created successfully."<<endl;
        m_bSimulationRunning=true;
        pthread_join(pth_Haptic,NULL);
    }
    else
    {
        cout<<"Create Thread Failed!"<<endl;
        hfdClose();
        return 0;
    }



    printf("main thread exit!\n");

    return a.exec();
}

void* updateHaptics(void*)
{
    while(!m_bSimulationFinished)
    {
        if(m_bOpen && m_bInit && m_bCalibrate)
        {
            hfdGetPosition(&m_Pos[0],&m_Pos[1],&m_Pos[2]);

            hfdGetOrientationFrame(m_Orientation);

            hfdGetAngularVelocityDeg(&m_AngVelDeg[0],&m_AngVelDeg[1],&m_AngVelDeg[2]);

            hfdGetJointAngles(m_JointAngRad);

            if(m_bForceEnable && m_bDeviceEnable)
            {
                hfdSetForce(m_Force[0],m_Force[1],m_Force[2]);
            }

            m_fButtonPress=hfdGetButton(0);

            if(m_fButtonPress)
            {
                m_bSimulationFinished=true;
            }

            system("clear");
            cout<<"Pos:"<<m_Pos[0]<<","<<m_Pos[1]<<","<<m_Pos[2]<<endl;
            cout<<"Orientation:"<<endl;
            cout<<m_Orientation[0][0]<<","<<m_Orientation[0][1]<<","<<m_Orientation[0][2]<<endl;
            cout<<m_Orientation[1][0]<<","<<m_Orientation[1][1]<<","<<m_Orientation[1][2]<<endl;
            cout<<m_Orientation[2][0]<<","<<m_Orientation[2][1]<<","<<m_Orientation[2][2]<<endl;
            cout<<"AngluarVelocityDeg:"<<m_AngVelDeg[0]<<","<<m_AngVelDeg[1]<<","<<m_AngVelDeg[2]<<endl;
            cout<<"JointAngleRad:"<<m_JointAngRad[0]<<","<<m_JointAngRad[1]<<","<<m_JointAngRad[2]
               <<","<<m_JointAngRad[3]<<","<<m_JointAngRad[4]<<","<<m_JointAngRad[5]<<endl;

        }
    }
    m_bSimulationRunning=false;
    hfdClose();
    cout<<"Haptic Thread exit!"<<endl;
    return 0;
}


