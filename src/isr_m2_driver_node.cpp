#include <ros/ros.h>
#include "isr_m2_driver/RobotStatus.h"
#include "isr_m2_driver/EncoderValue.h"
#include "isr_m2_driver/SetWheelVelocity.h"
#include "isr_m2_driver/RobotStatusStamped.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "isr_m2.h"
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

	// instanciate robot instance:
	rdrive::isr_m2::ISR_M2 isr_m2;
	isr_m2.Initialize();

    // subscribe to cmd_vel topic:
    ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel", 10, onCmdVel);

    // create robot status publisher:
    ros::Publisher pub_robot_status = nh.advertise<isr_m2_driver::RobotStatusStamped>("robot_status", 1, true);

    // instanciate a robot_status service proxy:
	ros::ServiceClient robot_status_client = nh.serviceClient<isr_m2_driver::RobotStatus>("robot_status");
	isr_m2_driver::EncoderValue encoder_value;

	// create odometry transform broadcaster:
	tf::TransformBroadcaster tf_odom_broadcaster;

	// create odometry publisher:
    ros::Publisher pub_odometry = nh.advertise<nav_msgs::Odometry>("odom", 10);

	// instanciate a read_encoder service proxy:
	ros::ServiceClient encoder_value_client = nh.serviceClient<isr_m2_driver::EncoderValue>("encoder_value");

    // wait until the communication is established:
    isr_m2_driver::RobotStatus robot_status, robot_status_old;
    bool robot_status_changed = false;
    ROS_INFO("Waiting for robot_status() service is ready...");
    ros::service::waitForService("robot_status", ros::Duration(-1));
    ROS_INFO("robot_status() service is ready.");
    ros::Duration(0.5).sleep(); // sleep for half a second
    if (robot_status_client.call(robot_status))
    {
        ROS_INFO("Robot Status: %d %d %d"
            , robot_status.response.motor_enabled
            , robot_status.response.motor_stopped
            , robot_status.response.estop_pressed);
        robot_status_old = robot_status;
    }
    else
    {
        ROS_ERROR("Failed to call service robot_status. This process will be shutdown.");
        return -1;
    }

    // set loop rate as 10hz
    ros::Rate loop_rate(10);
    ros::Time start_time = ros::Time::now();

    // main-loop:
    while (ros::ok())
    {
		// update robot_status and publish it if changed:
        if (robot_status_client.call(robot_status))
        {
            /*ROS_INFO("Robot Status: %d %d %d"
                , robot_status.response.motor_enabled
                , robot_status.response.motor_stopped
                , robot_status.response.estop_pressed);*/
            if (robot_status_old.response.motor_enabled != robot_status.response.motor_enabled)
            {
                ROS_INFO("Motor is %s", (robot_status.response.motor_enabled ? "ON" : "OFF"));
                robot_status_changed = true;
            }
            if (robot_status_old.response.motor_stopped != robot_status.response.motor_stopped)
            {
                ROS_INFO("Motor is %s", (robot_status.response.motor_stopped ? "Stopped" : "Resumed"));
                robot_status_changed = true;
            }
            if (robot_status_old.response.estop_pressed != robot_status.response.estop_pressed)
            {
                ROS_INFO("E-Stop button is %s", (robot_status.response.estop_pressed ? "Pressed" : "Released"));
                robot_status_changed = true;
            }

            if (robot_status_changed)
            {
                isr_m2_driver::RobotStatusStamped msg;
                msg.header.stamp = ros::Time::now();
                msg.motor_enabled = robot_status.response.motor_enabled;
                msg.motor_stopped = robot_status.response.motor_stopped;
                msg.estop_pressed = robot_status.response.estop_pressed;
                pub_robot_status.publish(msg);

                robot_status_old = robot_status;
                robot_status_changed = false;
            }
        }
        else
        {
            ROS_ERROR("Failed to call service robot_status or the service is not ready.");
        }

		// estimate robot pose and publish it:
		if (encoder_value_client.call(encoder_value))
        {
			ros::Time cur_time = ros::Time::now();

			// TODO: get exact time from DSP!
			isr_m2.SetEncoderValue(
				encoder_value.response.l_pulse_count, 
				encoder_value.response.r_pulse_count, 
                (int)((cur_time-start_time).toSec()*1000)/*encoder_value.response.time_ms*/);

            /*ROS_INFO("Odom: %d %d %d"
                     , encoder_value.response.l_pulse_count
                     , encoder_value.response.r_pulse_count
                     , (int)((cur_time-start_time).toSec()*1000));*/

			// broadcast odometry transform
			geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(isr_m2.Position.theta);
			geometry_msgs::TransformStamped odom_trans;
			odom_trans.header.stamp = cur_time;
			odom_trans.header.frame_id = "odom";
			odom_trans.child_frame_id = "isr_m2";
			odom_trans.transform.translation.x = isr_m2.Position.x;
			odom_trans.transform.translation.y = isr_m2.Position.y;
			odom_trans.transform.translation.z = 0.0;
			odom_trans.transform.rotation = odom_quat;
			tf_odom_broadcaster.sendTransform(odom_trans);

			// publish odometry message
			nav_msgs::Odometry odom;
            odom.header.stamp = cur_time;
			odom.header.frame_id = "odom";
			odom.child_frame_id = "isr_m2";
			odom.pose.pose.position.x = isr_m2.Position.x;
			odom.pose.pose.position.y = isr_m2.Position.y;
            odom.pose.pose.position.z = 0;
			odom.pose.pose.orientation = odom_quat;
            odom.pose.covariance.fill(0);
            odom.twist.twist.linear.x = 0;
            odom.twist.twist.linear.y = 0;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = 0;
            odom.twist.covariance.fill(0);
            pub_odometry.publish(odom);
		}
		else
		{
			ROS_ERROR("Failed to call service encoder_value or the service is not ready.");
		}

        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}
