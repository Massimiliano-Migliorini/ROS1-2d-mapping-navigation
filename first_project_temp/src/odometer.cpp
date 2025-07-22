#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <ros/master.h>
#include <vector>
#include <string>

bool topicExists(const std::string& topic_name) {

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (const auto& topic : topics) 
        if (topic.name == topic_name) 
            return true;
        
    return false;
}

// Topic /speedsteer
// y -> speed [km/h] 
// x -> steering wheel angle [°] 
//
//
//
//


class Odometer {

    private:

        ros::NodeHandle n;
        ros::NodeHandle n_p;

        // Subscribers for the "/speedsteer topic
        ros::Subscriber speedsteer_sub;
    
        // Publisher to the /odom topic
        ros::Publisher odom_pub;
    
        // Timer for periodic publishing
        ros::Timer timer;

        // Latest messages to be published on /odom topic
        nav_msgs::Odometry odom;

        // Latest received messages from the /speedsteer topics
        geometry_msgs::PointStamped msg_x_y;

        tf::TransformBroadcaster odom_br;
        tf::Transform odom_tr;  // to define the transformation (translation + orientation)
        tf::Quaternion q;
        
        double alpha, V, ome, theta_new, theta, x, y, factor, range, d, fs;
        bool start = false;
        
    public:

        // Constructor: sets up parameters, subscribers, publisher, and timer
        Odometer(): n(), n_p("~") {

            // Initialitation of global and private parameters in class attributes
            n.getParam("/front_to_rear", d);
            n.getParam("/steering_factor", factor);
            n.getParam("theta_init", theta);
            n_p.getParam("odom_sampling_f", fs);
            n_p.getParam("range", range);
            n_p.getParam("y_init", y);
            n_p.getParam("x_init", x);

            // Subscribe to /speedsteer
            speedsteer_sub = n.subscribe("/speedsteer", 1, &Odometer::SpeedSteerCallback, this);
    
            // Advertise the publisher on "/odom" topic
            odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1);
    
            // Create a timer that triggers at fs publishing on /odom
            timer = n.createTimer(ros::Duration(1.0/fs), &Odometer::timerCallback, this);

            tf::TransformBroadcaster odom_broadcaster_;
        }
    
        void SpeedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

            start = true; 


            msg_x_y = *msg; 
            V = msg_x_y.point.y/3.6; // speed [m/s]
            alpha = msg_x_y.point.x*M_PI/180.0; // steering [rad]

            // ROS_INFO("Speed: %lf, Steering: %lf, Odometer callback executed",V, alpha);

        }

        void timerCallback(const ros::TimerEvent&) {

            if(!topicExists("/speedsteer")) return;

            if (!start) return;

            // BICYCLE MODEL 
            ome = V*tan(alpha/factor)/d;

            // ODOMETRY INTEGRATION
            theta_new = theta + ome/fs;  // exact integration (assuming omega costant at each step)

            if (fabs(ome) > fabs(range)) {  // exact integration
                x += V/ome*(sin(theta_new) - sin(theta));
                y -= V/ome*(cos(theta_new) - cos(theta));
            }
            else {  // Runge-Kutta
                x += V/fs*(cos(theta + ome/(2*fs)));
                y += V/fs*(sin(theta + ome/(2*fs)));
            }

            theta = theta_new;
            
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_new);

            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "odom";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;
            odom.child_frame_id = "vehicle";
            odom.twist.twist.linear.x = V*cos(theta_new);
            odom.twist.twist.linear.y = V*sin(theta_new);
            odom.twist.twist.angular.z = ome; 
            
            odom_pub.publish(odom);

            // ROS_INFO("Odometry: x: %lf, y: %lf, theta: %lf", x, y, theta_new);

            // Update the transform's origin with the new pose
            odom_tr.setOrigin(tf::Vector3(x, y, 0));
            // Update the quaternion based on the new theta value
            q.setRPY(0, 0, theta_new); // We define quaternions from RPY angles of the message
            odom_tr.setRotation(q);
            // Publish the updated transform + header using the broadcaster
            odom_br.sendTransform(tf::StampedTransform(odom_tr, ros::Time::now(), "odom", "vehicle"));

        }
    
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "Odometer");
    Odometer node;
    ros::spin();
    return 0;

}
