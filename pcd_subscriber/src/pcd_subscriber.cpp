
#include <memory>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;


class PclSubPub : public rclcpp::Node
{

public:
    PclSubPub(std::string name)
        : Node(name)
    {
        
        // sub_left = this->create_subscription<sensor_msgs::msg::PointCloud2>("/realsense_left/camera/depth/color/points", 10, std::bind(&PclSubPub::sub_callback_l, this, std::placeholders::_1));
            // sub_middle = this->create_subscription<sensor_msgs::msg::PointCloud2>("/realsense_middle/camera/depth/color/points", 10, std::bind(&PclSubPub::sub_callback_m, this, std::placeholders::_1));
            sub_right = this->create_subscription<sensor_msgs::msg::PointCloud2>("realsense_right/depth/color/points", 10, std::bind(&PclSubPub::sub_callback, this, std::placeholders::_1));

        pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud2_unity", 10);
        // command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&PclSubPub::pub_callback, this));
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_left;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_middle;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_right;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
    // rclcpp::Publisher<pcl::PointXYZRGB>::SharedPtr pub;
    
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;


    void sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        //  RCLCPP_INFO(this->get_logger(),"loop start");
	    // pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);ss
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr
        pcl::fromROSMsg(*msg, *cloud);

        // rclcpp::TimerBase::SharedPtr timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PclSub::save_pcd_file("pcl_data_0.pcd", cloud), this, std::placeholders::_2) );
        // save_pcd_file("pcl_data_2.pcd", cloud);
        
        // RCLCPP_INFO(this->get_logger(), "points_size(%d,%d)",msg->height,msg->width);   

        // RCLCPP_INFO(this->get_logger(),"start declaration");
        sensor_msgs::msg::PointCloud2 msg_pub;
        // RCLCPP_INFO(this->get_logger(),"start convert");
        pcl::toROSMsg(*cloud, msg_pub);
        // RCLCPP_INFO(this->get_logger());
        // RCLCPP_INFO(this->get_logger(),"end convert, start publish" );
        pub -> publish(msg_pub);
        // RCLCPP_INFO(this->get_logger(),"loop end");
        // pub -> publish(*msg);

    };

    // void sub_callback_m(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    // {
    //     //todo
    // }

    // void sub_callback_r(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    // {
    //     //todo
    // }

    // void pub_callback()
    // {   
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = sub_callback_l();
    //     sensor_msgs::msg::PointCloud2::SharedPtr msg;
    //     pcl::toROSMsg(*cloud, *msg);

    //     pub -> publish(msg);
    // }
    
    void save_pcd_file(std::string name, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        pcl::io::savePCDFileASCII (name , *cloud);

        return;
    }
//     void save_pcd_file(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
//     {
//         pcl::io::savePCDFileASCII ("name" , *cloud);
//     }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PclSubPub>("pcl_sub_pub");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
