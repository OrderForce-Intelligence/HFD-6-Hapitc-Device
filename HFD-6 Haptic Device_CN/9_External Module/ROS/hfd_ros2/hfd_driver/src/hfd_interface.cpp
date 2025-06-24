#include "hfd_interface.h"


using namespace std;
using namespace std::chrono_literals;

bool m_bOpen = false;
bool m_bInit = false;
bool m_bCalibrate = false;
bool m_bForceEnable = false;
bool m_bExpertEnable = false;
bool m_bDeviceEnable = false;
bool m_bGravityCompensationEnable = false;

double m_Position[3] = {0};
double m_OrientationRad[3] = {0};
double m_OrientationFrame[3][3] = {0};
double m_LinearVelocity[3] = {0};
double m_AngularVelocityRad[3] = {0};
double m_Force[3] = {0};
double m_Torque[3] = {0};
bool m_ButtonPressed = false;
double m_ApplyForce[3] = {0};
double m_ApplyTorque[3] = {0};


/* Signal handler function for handling Ctrl+C signal. */
void sigintHandler(int sig) {
    if (m_bOpen) {
        hfdClose();
        m_bOpen = false;
        m_bExpertEnable = false;
        m_bDeviceEnable = false;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "HFD haptic device has been closed successfully!");
    rclcpp::shutdown();
}

/* Callback function for configuring hfd haptic device. */
int configureHFD(void) {
    // Check whether hfd haptic device is connected
    if (hfdGetDeviceCount() == 1) {
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "HFD haptic device detected!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "No hfd haptic device detected!");
        return 0;
    }
    
    // Open hfd haptic device
    if (hfdOpen() == 1) {
        m_bOpen = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Open hfd haptic device successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Open hfd haptic device failed!");
        return 0;
    }
    
    // Initialize hfd haptic device
    if (hfdInit() == 1) {
        m_bInit = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Initialize hfd haptic device successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Initialize hfd haptic device failed!");
        return 0;
    }
    
    // Calibrate hfd haptic device
    if (hfdCalibrateDevice() == 1) {
        m_bCalibrate = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Calibrate hfd haptic device successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Calibrate hfd haptic device failed!");
        return 0;
    }
    
    // Enable force mode for hfd haptic device
    if (hfdEnableForce(HFD_ON) == 1) {
        m_bForceEnable = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Enable force mode for hfd haptic device successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Enable force mode for hfd haptic device failed!");
        return 0;
    }
    
    // Enable expert mode for hfd haptic device
    if (hfdEnableExpertMode() == 1) {
        m_bExpertEnable = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Enable expert mode for hfd haptic device successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Enable expert mode for hfd haptic device failed!");
        return 0;
    }
    
    // Enable hfd haptic device for force control
    if (hfdEnableDevice(HFD_TRUE) == 1) {
        m_bDeviceEnable = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Enable hfd haptic device for force control successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Enable hfd haptic device for force control failed!");
        return 0;
    }
    
    // Enable gravity compensation for hfd haptic device
    if (hfdSetGravityCompensation(HFD_ON) == 1) {
        m_bGravityCompensationEnable = true;
        RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "Enable gravity compensation for hfd haptic device successfully!");
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Enable gravity compensation for hfd haptic device failed!");
        return 0;
    }
    
    // Return success
    RCLCPP_INFO(rclcpp::get_logger("hfd_interface"), "HFD haptic device has been configured and turned on successfully!");
    return 1;
}

/* Callback function for querying current state of hfd haptic device. */
int queryHFDState(void) {
    if (hfdGetPosition(&m_Position[0], &m_Position[1], &m_Position[2]) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Get current position of hfd stylus failed!");
        return 0;
    }
    
    if (hfdGetOrientationRad(&m_OrientationRad[0], &m_OrientationRad[1], &m_OrientationRad[2]) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Get current orientation radian of hfd stylus failed!");
        return 0;
    }
    
    if (hfdGetOrientationFrame(m_OrientationFrame) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Get current orientation frame of hfd stylus failed!");
        return 0;
    }
    
    if (hfdGetLinearVelocity(&m_LinearVelocity[0], &m_LinearVelocity[1], &m_LinearVelocity[2]) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Get current linear velocity of hfd stylus failed!");
        return 0;
    }
    
    // Enable expert mode again for fixing bugs of hfdGetAngularVelocityRad failure
    hfdEnableExpertMode();
    
    if (hfdGetAngularVelocityRad(&m_AngularVelocityRad[0], &m_AngularVelocityRad[1], &m_AngularVelocityRad[2]) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Get current angular velocity radian of hfd stylus failed!");
        return 0;
    }
    
    if (hfdGetForceAndTorque(&m_Force[0], &m_Force[1], &m_Force[2], &m_Torque[0], &m_Torque[1], &m_Torque[2]) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Get current force and torque of hfd stylus failed!");
        return 0;
    }
    
    if (hfdGetButton(0) == 1) { m_ButtonPressed = true; } else { m_ButtonPressed = false; }
    
    // Return success
    return 1;
}

/* Callback function for applying force to hfd haptic device. */
void applyHFDForce(const xjcsensor_msgs::msg::FTSensorState & msg) {
    // Handle the apply force message
    m_ApplyForce[0] = msg.wrench.force.x; // Unit: N
    m_ApplyForce[1] = msg.wrench.force.y;
    m_ApplyForce[2] = msg.wrench.force.z;
    m_ApplyTorque[0] = msg.wrench.torque.x * 1000; // Unit: N*mm
    m_ApplyTorque[1] = msg.wrench.torque.y * 1000;
    m_ApplyTorque[2] = msg.wrench.torque.z * 1000;
    
    // Apply force to hfd haptic device
    if (hfdSetForce(m_ApplyForce[0], m_ApplyForce[1], m_ApplyForce[2]) == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Apply force to hfd haptic device failed!");
    }
    // if (hfdSetForceAndTorque(m_ApplyForce[0], m_ApplyForce[1], m_ApplyForce[2], m_ApplyTorque[0], m_ApplyTorque[1], m_ApplyTorque[2]) == 0) {
    //     RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Apply force and torque to hfd haptic device failed!");
    // }
}


/* Main function for running hfd interface node. */
int main(int argc, char *argv[]) {
    // Set up locale for the program
    setlocale(LC_ALL, "");
    
    // Initialize ros node
    rclcpp::init(argc, argv);
    auto nh = rclcpp::Node::make_shared("hfd_interface");
    
    // Set up publisher for hfd haptic device state and subscriber for hfd haptic device applied force
    auto pub = nh->create_publisher<hfd_msgs::msg::HFDState>("/hfd/device_state", 10);
    auto sub = nh->create_subscription<xjcsensor_msgs::msg::FTSensorState>("/xjcsensor/ft_sensor_state", 10, applyHFDForce);
    
    // Define ros message for hfd haptic device
    hfd_msgs::msg::HFDState hfd_state;
    
    // Set up signal handler for handling Ctrl+C signal
    signal(SIGINT, sigintHandler);
    
    // Configure hfd haptic device
    if (configureHFD() == 0) {
        RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Configure and turn on hfd haptic device failed!");
        return -1;
    }
    
    // Run the main loop
    while (rclcpp::ok()) {
        // Query current state of hfd haptic device
        if (queryHFDState() == 0) {
            RCLCPP_ERROR(rclcpp::get_logger("hfd_interface"), "Query current state of hfd haptic device failed!");
            return -1;
        }
        
        // Set up hfd message with current state of hfd haptic device
        hfd_state.header.stamp = nh->now();
        hfd_state.header.frame_id = "hfd";
        hfd_state.pose.position.x = m_Position[0] * 0.001; // Unit: m
        hfd_state.pose.position.y = m_Position[1] * 0.001;
        hfd_state.pose.position.z = m_Position[2] * 0.001;
        hfd_state.orientation.x = m_OrientationRad[0]; // Unit: rad
        hfd_state.orientation.y = m_OrientationRad[1];
        hfd_state.orientation.z = m_OrientationRad[2];
        hfd_state.velocity.linear.x = m_LinearVelocity[0] * 0.001; // Unit: m/s
        hfd_state.velocity.linear.y = m_LinearVelocity[1] * 0.001;
        hfd_state.velocity.linear.z = m_LinearVelocity[2] * 0.001;
        hfd_state.velocity.angular.x = m_AngularVelocityRad[0]; // Unit: rad/s
        hfd_state.velocity.angular.y = m_AngularVelocityRad[1];
        hfd_state.velocity.angular.z = m_AngularVelocityRad[2];
        hfd_state.wrench.force.x = m_Force[0]; // Unit: N
        hfd_state.wrench.force.y = m_Force[1];
        hfd_state.wrench.force.z = m_Force[2];
        hfd_state.wrench.torque.x = m_Torque[0] * 0.001; // Unit: N*m
        hfd_state.wrench.torque.y = m_Torque[1] * 0.001;
        hfd_state.wrench.torque.z = m_Torque[2] * 0.001;
        hfd_state.button_status = m_ButtonPressed;
        
        // Print out the current state of hfd haptic device
        RCLCPP_INFO(nh->get_logger(), "HFD stylus position: x: %f, y: %f, z: %f", hfd_state.pose.position.x, hfd_state.pose.position.y, hfd_state.pose.position.z);
        RCLCPP_INFO(nh->get_logger(), "HFD stylus orientation: rx: %f, ry: %f, rz: %f", hfd_state.pose.orientation.x, hfd_state.pose.orientation.y, hfd_state.pose.orientation.z);
        RCLCPP_INFO(nh->get_logger(), "HFD stylus linear velocity: vx: %f, vy: %f, vz: %f", hfd_state.velocity.linear.x, hfd_state.velocity.linear.y, hfd_state.velocity.linear.z);
        RCLCPP_INFO(nh->get_logger(), "HFD stylus angular velocity: vrx: %f, vry: %f, vrz: %f", hfd_state.velocity.angular.x, hfd_state.velocity.angular.y, hfd_state.velocity.angular.z);
        RCLCPP_INFO(nh->get_logger(), "HFD stylus force: fx: %f, fy: %f, fz: %f", hfd_state.wrench.force.x, hfd_state.wrench.force.y, hfd_state.wrench.force.z);
        RCLCPP_INFO(nh->get_logger(), "HFD stylus torque: tx: %f, ty: %f, tz: %f", hfd_state.wrench.torque.x, hfd_state.wrench.torque.y, hfd_state.wrench.torque.z);
        RCLCPP_INFO(nh->get_logger(), "HFD stylus button status: %d", hfd_state.button_status);
        cout << endl;
        
        // Publish hfd message from hfd haptic device
        pub->publish(hfd_state);
        
        // Spin the ros node and sleep for a while
        rclcpp::spin_some(nh);
        rclcpp::sleep_for(2ms);
    }
    
    // Shutdown ros node
    rclcpp::shutdown();
    
    // Exit the program
    return 0;
}
