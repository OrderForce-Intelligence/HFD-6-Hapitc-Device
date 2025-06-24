#include <ros/ros.h>
#include <signal.h>

#include "HFD_OPEN.h"
#include "hfdDefines.h"
#include "hfdVector.h"
#include "hfd_driver/xd.h"

using namespace std;

bool m_bOpen;
bool m_bInit;
bool m_bCalibrate;
bool m_bForceEnable;
bool m_bDeviceEnable;

bool m_bSimulationRunning;
bool m_bSimulationFinished;

double m_Pos[3] = {0};
double m_Orientation[3][3] = {0};
int m_fButtonPress = false;
double m_Force[3] = {0};

// 信号处理函数，用于捕获 Ctrl+C 信号
void sigintHandler(int sig) {
    ROS_INFO("Shutting down the hfd device...");
    if (m_bOpen) {
        hfdClose();
        m_bOpen = false;
    }
    ros::shutdown();
}
int main(int argc, char* argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "claf_control", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<hfd_driver::xd>("/arm_claf_data", 10);
    ros::Duration(2).sleep();
    ROS_INFO("openg and initail arm remote device.");
    // 设置信号处理函数
    signal(SIGINT, sigintHandler);
    m_bOpen = false;
    m_bInit = false;
    m_bCalibrate = false;
    m_bForceEnable = false;
    m_bDeviceEnable = false;
    m_bSimulationRunning = false;
    m_bSimulationFinished = false;

    int m_DeviceCount = hfdGetDeviceCount();
    if (!m_DeviceCount) {
        cout << "No hfd Haptic Device Detected!" << endl;
        return 0;
    }

    int error = 0;

    error = hfdOpen();
    if (error < 0) {
        cout << "Open the hfd Haptic Device Failed!" << endl;
        return 0;
    }
    m_bOpen = true;

    error = hfdInit();
    if (error == 0) {
        cout << "Init the hfd Haptic Device Failed!" << endl;
        return 0;
    }
    m_bInit = true;

    error = hfdCalibrateDevice();
    if (error == 0) {
        cout << "Calibrate the hfd Haptic Device Failed!" << endl;
        return 0;
    }
    m_bCalibrate = true;

    error = hfdEnableForce(HFD_ON);
    if (error == 0) {
        cout << "Failed to Enable Force Mode!" << endl;
        return 0;
    }
    m_bForceEnable = true;

    hfdEnableExpertMode();

    error = hfdEnableDevice(true);
    if (error == 0) {
        cout << "Failed to Enable Haptice Device!" << endl;
        return 0;
    }
    m_bDeviceEnable = true;

    error = hfdSetGravityCompensation(HFD_ON);
    if (error == 0) {
        cout << "Failed to Set Gravity Compensation!" << endl;
        return 0;
    }
    ROS_INFO("hfd device has been turned on.\n");

    hfd_driver::xd xd01;
    double pos[9] = {0};
    int hfdButton = 0;
    ros::Rate rate(4000);

    while (ros::ok()) {
        hfdGetLinearVelocity(&pos[0], &pos[1], &pos[2]);
        hfdGetAngularVelocityDeg(&pos[3], &pos[4], &pos[5]);
        hfdGetPosition(&pos[6], &pos[7], &pos[8]);
        xd01.hfdButtonStatus = hfdGetButton(hfdButton);
        xd01.vx = pos[0] * 0.001;
        xd01.vy = pos[1] * 0.001;
        xd01.vz = pos[2] * 0.001;
        xd01.rx = pos[3];
        xd01.ry = pos[4];
        xd01.rz = pos[5];
        xd01.x  = pos[6];
        xd01.y  = pos[7];
        xd01.z  = pos[8];
        // 输出数据
        ROS_INFO("Linear  Velocity: vx: %f, vy: %f, vz: %f", xd01.vx, xd01.vy, xd01.vz);
        ROS_INFO("Angular Velocity: rx: %f, ry: %f, rz: %f", xd01.rx, xd01.ry, xd01.rz);
        ROS_INFO("device  position:  x: %f,  y: %f,  z: %f", xd01.x, xd01.y, xd01.z);
        ROS_INFO("Button Status: %d", xd01.hfdButtonStatus);

        pub.publish(xd01);
        rate.sleep();
        ros::spinOnce();
    }
    m_bSimulationRunning = false;
    return 0;
}
