#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "nav_msgs/Odometry.h"
#include "first_project/sector_times.h"
#include "sensor_msgs/NavSatFix.h"
#include <cmath>
#include <vector>
#include <string>
#include <ros/master.h>


bool topicExists(const std::string& topic_name) {

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (const auto& topic : topics) 
        if (topic.name == topic_name) 
            return true;
        
    return false;
}


class Sector_Times {
    private:
        // ROS node handle
        ros::NodeHandle n;
        ros::NodeHandle n_p;

        // Subscribers for the /speed_steer_sub
        ros::Subscriber speed_steer_sub;
        // Subscriber for the /swiftnav/front/gps_pose
        ros::Subscriber front_gps_sub;
    
        // Publisher 
        ros::Publisher sector_times_pub;
    
        // Timer for periodic publishing
        ros::Timer timer_;

        // Latest messages to be published on /sector_times topic
        first_project::sector_times msg_sector;

        // Latest received messages from the /speedsteer topics
        geometry_msgs::PointStamped msg_x_y;
        // Latest received messages from the /swiftnav/front/gps_pose topics
        sensor_msgs::NavSatFix msg_lla;

        double fs, s1_lat, s1_lon, s1_lon_l, s2_lat, s2_lon, s3_lat, s3_lon, v_sum = 0, t, t_enter, V, lat, lon;
        int sector = 1, i = 0;  // da generalizzare...
        double map_limit_upper_lat, map_limit_upper_lon, map_limit_lower_lat, map_limit_lower_lon;
        bool k = true, speed_data_ready = false;
        

        
        
    public:
        // Constructor: sets up parameters, subscribers, publisher, and timer
        Sector_Times(): n(), n_p("~") {
            // Initialitation of global and private parameters

            n_p.getParam("sector_sampling_f", fs);
            n_p.getParam("sector_1_lat", s1_lat);
            n_p.getParam("sector_1_lon_lim", s1_lon_l);
            n_p.getParam("sector_1_lon", s1_lon);
            n_p.getParam("sector_2_lat", s2_lat);
            n_p.getParam("sector_2_lon", s2_lon);
            n_p.getParam("sector_3_lat", s3_lat);
            n_p.getParam("sector_3_lon", s3_lon);

            n_p.getParam("map_limit_lat_top_right_corner", map_limit_upper_lat);
            n_p.getParam("map_limit_lon_top_right_corner", map_limit_upper_lon);
            n_p.getParam("map_limit_lat_bottom_left_corner", map_limit_lower_lat);
            n_p.getParam("map_limit_lon_bottom_left_corner", map_limit_lower_lon);


            // Subscribe to topics with a queue size of 1
            speed_steer_sub = n.subscribe("/speedsteer", 1, &Sector_Times::SpeedSteerCallback, this);
            front_gps_sub = n.subscribe("/swiftnav/front/gps_pose", 1, &Sector_Times::FrontGpsCallback, this);
    
            // Advertise the publisher on "/sector_times" topic
            sector_times_pub = n.advertise<first_project::sector_times>("/sector_times", 1);
    
            // Create a timer that triggers every 1 second to publish messages
            timer_ = n.createTimer(ros::Duration(1.0/fs), &Sector_Times::timerCallback, this);

        }
    
        // Callback function for the "/chatter" topic

        
    // Callback function for the "/chatter" topic
    void SpeedSteerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        msg_x_y = *msg; 
        V = msg_x_y.point.y; // speed [km/h]
        t = msg_x_y.header.stamp.toSec();
        speed_data_ready = true;

    }

    void FrontGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {

        msg_lla = *msg; 
        lat = msg_lla.latitude;
        lon = msg_lla.longitude;

    }
    

    // Timer callback function: publishes messages periodically
    void timerCallback(const ros::TimerEvent&) {

        if(!topicExists("/speedsteer")) return;

        if (!speed_data_ready) return;

        if (k == true) {
            t_enter = t;
            k = false;
        }
        else if (sector == 3 && lat >= s1_lat && lon <= s1_lon_l && (lat<=map_limit_upper_lat && lat>=map_limit_lower_lat) && (lon<=map_limit_upper_lon && lon>=map_limit_lower_lon)) {
            reset(1);
        }
        else if (sector == 1 &&  lon >= s2_lon && (lon<=map_limit_upper_lon && lon>=map_limit_lower_lon) && (lat<=map_limit_upper_lat && lat>=map_limit_lower_lat)) {
            reset(2);
        }
        else if(sector == 2 && lat <= s3_lat && (lat<=map_limit_upper_lat && lat>=map_limit_lower_lat) && (lon<=map_limit_upper_lon && lon>=map_limit_lower_lon)) {
            reset(3);
        }
        else {
            i++;
            v_sum += V;
            msg_sector.current_sector = sector;
            msg_sector.current_sector_time = t - t_enter;
            msg_sector.current_sector_mean_speed = v_sum/i;

            sector_times_pub.publish(msg_sector);
        }

    

    }
    void reset(int sec) {
        sector = sec;
        t_enter = t;
        v_sum = 0;
        i = 0;
    }
    
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "Odometer");
    Sector_Times node;
    ros::spin();
    return 0;
}
