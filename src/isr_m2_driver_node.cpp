#include "ros/ros.h"
#include "isr_m2_driver/RobotStatus.h"
#include "isr_m2_driver/SetWheelVelocity.h"
#include "isr_m2_driver/RobotStatusStamped.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib>

#define WHEEL_RADIUS_M  0.155   // m
#define WHEEL_BASE_M    0.531   // m
#define WHEEL_WIDTH_M   0.102   // m
#define ENCODER_PPR     6400.0  // pulse/rev
#define GEAR_RATIO      31.778  // gearhead reduction ratio = 26 (26:1), spurgear reduction ratio = 1.22 (44:36)
#define MPS2RPM         61.608  // 60 / (2 * M_PU * WHEEL_RADIUS_M)
#define MAX_RPM         4650.0

void onCmdVel(const geometry_msgs::Twist& cmd_vel)
{
    ROS_INFO("onCmdVel()");

    double linearVel_MPS = cmd_vel.linear.x;
    double angularVel_RPS = cmd_vel.angular.z;

    double leftWheelVel_MPS = linearVel_MPS - (WHEEL_BASE_M*angularVel_RPS / 2.);
    int leftMotorVel_RPM = (int)(leftWheelVel_MPS * MPS2RPM * GEAR_RATIO);

    double rightWheelVel_MPS = linearVel_MPS + (WHEEL_BASE_M*angularVel_RPS / 2.);
    int rightMotorVel_RPM = (int)(rightWheelVel_MPS * MPS2RPM * GEAR_RATIO);

    if (leftMotorVel_RPM > MAX_RPM || rightMotorVel_RPM > MAX_RPM ||
        leftMotorVel_RPM < -MAX_RPM || rightMotorVel_RPM < -MAX_RPM)
        return;

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<isr_m2_driver::SetWheelVelocity>("set_wheel_velocity");
    isr_m2_driver::SetWheelVelocity srv;
    srv.request.l_motor_vel_rpm = leftMotorVel_RPM;
    srv.request.r_motor_vel_rpm = rightMotorVel_RPM;
    if (client.call(srv))
    {
        ROS_INFO("set wheel velocity: %d %d"
            , leftMotorVel_RPM
            , rightMotorVel_RPM);
    }
    else
    {
        ROS_ERROR("Failed to call service set_wheel_velocity");
        return;
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "isr_m2_driver");
	ros::NodeHandle nh;

    // subscribe to cmd_vel topic:
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 10, onCmdVel);

    // create robot status publisher:
    ros::Publisher pub_robot_status = nh.advertise<isr_m2_driver::RobotStatusStamped>("robot_status", 1, true);

    // instanciate a service proxy:
	ros::ServiceClient client = nh.serviceClient<isr_m2_driver::RobotStatus>("robot_status");

    // wait until the communication is established:
    isr_m2_driver::RobotStatus srv, srv_old;
    bool srv_changed = false;
    ROS_INFO("Waiting for robot_status() service is ready...");
    ros::service::waitForService("robot_status", ros::Duration(-1));
    ROS_INFO("robot_status() service is ready.");
    ros::Duration(0.5).sleep(); // sleep for half a second
    if (client.call(srv))
    {
        ROS_INFO("Robot Status: %d %d %d"
            , srv.response.motor_enabled
            , srv.response.motor_stopped
            , srv.response.estop_pressed);
        srv_old = srv;
    }
    else
    {
        ROS_ERROR("Failed to call service robot_status. This process will be shutdown.");
        return -1;
    }

    // set loop rate as 10hz
    ros::Rate loop_rate(10);

    // main-loop:
    while (ros::ok())
    {
        if (client.call(srv))
        {
            /*ROS_INFO("Robot Status: %d %d %d"
                , srv.response.motor_enabled
                , srv.response.motor_stopped
                , srv.response.estop_pressed);*/
            if (srv_old.response.motor_enabled != srv.response.motor_enabled)
            {
                ROS_INFO("Motor is %s", (srv.response.motor_enabled ? "ON" : "OFF"));
                srv_changed = true;
            }
            if (srv_old.response.motor_stopped != srv.response.motor_stopped)
            {
                ROS_INFO("Motor is %s", (srv.response.motor_stopped ? "Stopped" : "Resumed"));
                srv_changed = true;
            }
            if (srv_old.response.estop_pressed != srv.response.estop_pressed)
            {
                ROS_INFO("E-Stop button is %s", (srv.response.estop_pressed ? "Pressed" : "Released"));
                srv_changed = true;
            }

            if (srv_changed)
            {
                isr_m2_driver::RobotStatusStamped msg;
                msg.header.stamp = ros::Time::now();
                msg.motor_enabled = srv.response.motor_enabled;
                msg.motor_stopped = srv.response.motor_stopped;
                msg.estop_pressed = srv.response.estop_pressed;
                pub_robot_status.publish(msg);

                srv_old = srv;
                srv_changed = false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service robot_status or the service is not ready.");
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}
