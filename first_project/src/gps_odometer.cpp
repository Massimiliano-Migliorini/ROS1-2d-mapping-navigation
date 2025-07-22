#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include "sensor_msgs/NavSatFix.h"
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

class Gps_Odometer {
    private:
        // ROS node handle
        ros::NodeHandle n;
        ros::NodeHandle n_p;

        // Subscriber for the /swiftnav/front/gps_pose
        ros::Subscriber front_gps_sub;
    
        // Publisher to /gps_odom
        ros::Publisher gps_odom_pub;
    
        // Timer for periodic publishing
        ros::Timer timer_;

        // Latest messages to be published on /odom topic
        nav_msgs::Odometry gps_odom;

        // Latest received messages from the /swiftnav/front/gps_pose topics
        sensor_msgs::NavSatFix msg_lla;

        tf::TransformBroadcaster gps_odom_br;
        tf::Transform gps_odom_tr;  // to define the transformation (translation + orientation)
        tf::Quaternion q;
        
        // 
        double fs, lat, lon, alt, a, b, X, Y, Z, x, y, z, x_odom, y_odom, z_odom, lat_r, lon_r, alt_r, X_r, Y_r, Z_r, theta, x_old = 0, y_old = 0, theta_static, theta_old, alpha1, alpha2;
        //double i=0, i_start=143, i_end=580, mean, j=0, sum=0;  //To find theta_static
        bool k = false, gps_data_ready = false;
    
        // frequency of odometry sampling (odometry publishing - timer frequency)
        
    
    public:
        // Constructor: sets up parameters, subscribers, publisher, and timer
        Gps_Odometer(): n(), n_p("~") {

            // Initialitation of global and private parameters
            n_p.getParam("theta_static", theta_static);
            n_p.getParam("gps_odom_sampling_f", fs);
            n_p.getParam("lat_r", lat_r);
            n_p.getParam("lon_r", lon_r);
            n_p.getParam("alt_r", alt_r);
            n_p.getParam("a", a);
            n_p.getParam("b", b);

            theta_old = theta_static;

            lat_r = lat_r*M_PI/180;
            lon_r = lon_r*M_PI/180;
            alt_r = alt_r;

            double e_2 = 1 - ((b*b) / (a*a));

            double N_r = a / sqrt(1-e_2*sin(lat_r)*sin(lat_r));

            X_r = (N_r + alt_r)*cos(lat_r)*cos(lon_r);
            Y_r = (N_r + alt_r)*cos(lat_r)*sin(lon_r);
            Z_r = (N_r*(1-e_2) + alt_r )*sin(lat_r);

            // Subscribe to topics with a queue size of 1
            front_gps_sub = n.subscribe("/swiftnav/front/gps_pose", 1, &Gps_Odometer::gps_to_odom_Callback, this);
    
            // Advertise the publisher on "/rechatter" topic
            gps_odom_pub = n.advertise<nav_msgs::Odometry>("/gps_odom", 1);
    
            // Create a timer that triggers every 1 second to publish messages
            timer_ = n.createTimer(ros::Duration(1.0/fs), &Gps_Odometer::timerCallback, this);

        }
    
        // Callback function for the "/chatter" topic

        
    // Callback function for the "/sensor_msgs/NavSatFix" topic
    void gps_to_odom_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        msg_lla = *msg; 
        lat = msg_lla.latitude*M_PI/180.0;
        lon = msg_lla.longitude*M_PI/180.0;
        alt = msg_lla.altitude;
        
        gps_data_ready = true;

        // ROS_INFO("Lat: %lf, Lon: %lf, Alt: %lf", lat, lon, alt);

    }

    

    // Timer callback function: publishes messages periodically
    void timerCallback(const ros::TimerEvent&) {

        if(!topicExists("/speedsteer")) return;

        if (!gps_data_ready) return;

        GPS_to_ECEF();

        ECEF_to_ENU();

        ENU_to_ODOM();

        Heading();

        // i++;

        // if(i>i_start && i < i_end) {
        //     j++;
        //     sum += theta*180/M_PI;
        // }
        // if (i > i_end) {
        //     mean = sum/j;
        //     ROS_INFO("Mean :%lf", mean);
        // }
           
        // 85.136787

        
           

        geometry_msgs::Quaternion gps_odom_quat = tf::createQuaternionMsgFromYaw(theta);
  
        gps_odom.header.stamp = ros::Time::now();
        gps_odom.header.frame_id = "odom";
        gps_odom.pose.pose.position.x = x_odom;
        gps_odom.pose.pose.position.y = y_odom;
        gps_odom.pose.pose.position.z = z_odom;
        gps_odom.pose.pose.orientation = gps_odom_quat;
        gps_odom.child_frame_id = "gps";
    
        gps_odom_pub.publish(gps_odom);

        //double theta_degree = theta*180/M_PI;

        //ROS_INFO("I heard x: %lf, y: %lf, z: %lf, theta: %lf, i:%lf", x, y, z, theta_degree, i);

        // Update the transform's origin with the new pose
        gps_odom_tr.setOrigin(tf::Vector3(x_odom, y_odom, z_odom));
        // Update the quaternion based on the new theta value
        q.setRPY(0, 0, theta); // We define quaternions from RPY angles of the message
        gps_odom_tr.setRotation(q);
        // Publish the updated transform + header using the broadcaster
        gps_odom_br.sendTransform(tf::StampedTransform(gps_odom_tr, ros::Time::now(), "odom", "gps"));

    }

    void GPS_to_ECEF(){

        double e_2 = 1 - ((b*b) / (a*a));
        double N = a / sqrt(1-e_2*sin(lat)*sin(lat));

        X = (N + alt)*cos(lat)*cos(lon);
        Y = (N + alt)*cos(lat)*sin(lon);
        Z = (N*(1-e_2) + alt )*sin(lat);


    }

    void ECEF_to_ENU(){

        double dx = X - X_r;
        double dy = Y - Y_r;
        double dz = Z - Z_r;
        

        x = -sin(lon_r) * dx + cos(lon_r) * dy;
        y = -sin(lat_r) * cos(lon_r) * dx - sin(lat_r) * sin(lon_r) * dy + cos(lat_r) * dz;
        z = cos(lat_r) * cos(lon_r) * dx + cos(lat_r) * sin(lon_r) * dy + sin(lat_r) * dz;


    }

    void ENU_to_ODOM() {
        x_odom = cos(theta_static)*x + sin(theta_static)*y;
        y_odom = - sin(theta_static)*x + cos(theta_static)*y;
        z_odom = z;
    }

    void Heading() {
        
        if (fabs(x - x_old) > 1e-1 || fabs(y - y_old) > 1e-1) {
            alpha1=0.8;
            alpha2=0.2;
        }
        else {
            alpha1=0;
            alpha2=1;
        }
        
        theta = alpha1*atan2(y - y_old, x - x_old) + alpha2*theta_old;
        theta_old = theta;
        theta -= theta_static;
        x_old = x;
        y_old = y;
    }


    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Gps_Odometer");
    Gps_Odometer node;
    ros::spin();
    return 0;
}

