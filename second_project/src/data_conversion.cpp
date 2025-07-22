#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/LaserScan.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


struct SensorPose
{
  double x {0.0}, y {0.0}, yaw {0.0};    // [m, m, rad]
};

bool topicExists(const std::string& topic_name) {

    ros::master::V_TopicInfo topics;
    ros::master::getTopics(topics);
    for (const auto& topic : topics) 
        if (topic.name == topic_name) 
            return true;
        
    return false;
}


class DataConverter {

    private:

        ros::NodeHandle n;
       
        ros::Subscriber odom_sub;

        ros::Publisher pub_front;
        ros::Publisher pub_back;

        message_filters::Subscriber<sensor_msgs::LaserScan> sub_front;
        message_filters::Subscriber<sensor_msgs::LaserScan> sub_back;
    
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
        message_filters::Synchronizer<MySyncPolicy> sync;

        ros::Timer odom_timer;

        nav_msgs::Odometry odom;
        sensor_msgs::LaserScan scan_front_msg;
        sensor_msgs::LaserScan scan_back_msg;

        geometry_msgs::TransformStamped tf1, tf4, tf3;
        tf2_ros::StaticTransformBroadcaster static_broadcaster;
        std::vector<geometry_msgs::TransformStamped> transforms;

        tf::TransformBroadcaster br;
        tf::Transform transform;  
        tf::Quaternion q;

        float f_odom = 20.0, inc;
        int indx_f_start = 95, indx_f_end = 710, indx_b_start = 95, indx_b_end = 715;

        bool odom_data_ready = false;

        SensorPose sf_{0.30, 0.00, 0.00}; 
        SensorPose sb_{-0.30, 0.00, 3.1416};
        
    public:

        DataConverter(): 
        sub_front(n, "/scan_front", 1),
        sub_back(n, "/scan_back",  1),
        sync(MySyncPolicy(10), sub_front, sub_back)
        {
     
            odom_sub = n.subscribe<nav_msgs::Odometry>("/odometry",1,&DataConverter::odom_callback,this);                 

            pub_front = n.advertise<sensor_msgs::LaserScan>("/filtered_scan_front", 1);
            pub_back = n.advertise<sensor_msgs::LaserScan>("/filtered_scan_back", 1);

            sync.setMaxIntervalDuration(ros::Duration(0.20));
            sync.registerCallback(boost::bind(&DataConverter::scan_callback, this, _1, _2));

            odom_timer = n.createTimer(ros::Duration(1.0/f_odom), &DataConverter::timer_callback_odom, this);
            
            tf::TransformBroadcaster odom_broadcaster_;

            tf1.header.stamp = ros::Time(0);
            tf1.header.frame_id = "base_link";
            tf1.child_frame_id = "body_link";
            tf1.transform.translation.x = 0.0;
            tf1.transform.translation.y = 0.0;
            tf1.transform.translation.z = 0.0;

            tf2::Quaternion q1;
            q1.setRPY(0, 0, 0);
            tf1.transform.rotation.x = q1.x();
            tf1.transform.rotation.y = q1.y();
            tf1.transform.rotation.z = q1.z();
            tf1.transform.rotation.w = q1.w();

            // base_link -> sick_back
            tf4.header.stamp = ros::Time(0);
            tf4.header.frame_id = "base_link";
            tf4.child_frame_id = "sick_back";
            tf4.transform.translation.x = -0.3;
            tf4.transform.translation.y = 0.0;
            tf4.transform.translation.z = -0.115;

            tf2::Quaternion q2;
            q2.setRPY(0, 0, 3.14159); // 180 degrees in radians
            tf4.transform.rotation.x = q2.x();
            tf4.transform.rotation.y = q2.y();
            tf4.transform.rotation.z = q2.z();
            tf4.transform.rotation.w = q2.w();

            // base_link -> sick_front
            tf3.header.stamp = ros::Time(0);
            tf3.header.frame_id = "base_link";
            tf3.child_frame_id = "sick_front";
            tf3.transform.translation.x = 0.3;
            tf3.transform.translation.y = 0.0;
            tf3.transform.translation.z = -0.115;

            tf2::Quaternion q3;
            q3.setRPY(-3.14159, 0, 0); 
            tf3.transform.rotation.x = q3.x();
            tf3.transform.rotation.y = q3.y();
            tf3.transform.rotation.z = q3.z();
            tf3.transform.rotation.w = q3.w();



            // Add to vector and broadcast
            transforms.push_back(tf1);
            transforms.push_back(tf4);
            transforms.push_back(tf3);
            static_broadcaster.sendTransform(transforms);


        }

        void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
            odom = *msg;
            odom_data_ready = true; 
        }

        void scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg_f, const sensor_msgs::LaserScan::ConstPtr& msg_b)
        {
            scan_front_msg = *msg_f;
            scan_back_msg  = *msg_b;

            inc = msg_f->angle_increment;
            scan_front_msg.angle_max = inc * indx_f_end + msg_f->angle_min; 
            scan_front_msg.angle_min = inc * indx_f_start + msg_f->angle_min;
            scan_back_msg.angle_max = inc * indx_b_end + msg_b->angle_min;
            scan_back_msg.angle_min = inc * indx_b_start + msg_b->angle_min;
            scan_front_msg.ranges.assign(msg_f->ranges.begin()+indx_f_start,
                             msg_f->ranges.begin()+indx_f_end+1);
            scan_back_msg.ranges.assign(msg_b->ranges.begin()+indx_b_start,
                             msg_b->ranges.begin()+indx_b_end+1);


            pub_front.publish(scan_front_msg);
            pub_back.publish(scan_back_msg);
            
        }
   

        void timer_callback_odom(const ros::TimerEvent&)
        {
        
        if(!topicExists("/odometry")) return;
        if (!odom_data_ready) return;
        // --- Traslazione -------------------------------------------------
        transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                        odom.pose.pose.position.y,
                                        odom.pose.pose.position.z));
        // --- Rotazione ---------------------------------------------------
        q.setX(odom.pose.pose.orientation.x);
        q.setY(odom.pose.pose.orientation.y);
        q.setZ(odom.pose.pose.orientation.z);
        q.setW(odom.pose.pose.orientation.w);
        transform.setRotation(q);

        // Pubblica la trasformazione con lo stesso timestamp del messaggio
        br.sendTransform(tf::StampedTransform(transform,
                                                odom.header.stamp, // tempo
                                                "odom",            // frame padre
                                                "base_link"));     // frame figlio
        }
        
};

int main(int argc, char **argv) {

    ros::init(argc, argv, "DataConverter");
    DataConverter node;
    ros::spin();
    return 0;

}




