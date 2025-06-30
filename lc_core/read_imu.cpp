#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip> 

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <input_bag_file> <output_txt_file>" << std::endl;
        return 1;
    }

    std::string bag_file = argv[1];
    std::string output_file = argv[2];
    std::string topic_name = "/livox/imu";

    try {
        std::ofstream outfile(output_file);
        if (!outfile.is_open()) {
            std::cerr << "Error: Could not open output file " << output_file << std::endl;
            return 1;
        }
        outfile << "# Timestamp(sec) "
                << "Accel_X Accel_Y Accel_Z "
                << "Gyro_X Gyro_Y Gyro_Z "
                << "Quat_X Quat_Y Quat_Z Quat_W"
                << std::endl;

        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);
        rosbag::View view(bag, rosbag::TopicQuery(topic_name));

        std::cout << "Found " << view.size() << " messages on topic " << topic_name << std::endl;
        std::cout << "Writing data to " << output_file << std::endl;
        outfile << std::fixed << std::setprecision(9);
        for (const rosbag::MessageInstance& msg : view) {
            sensor_msgs::Imu::ConstPtr imu_msg = msg.instantiate<sensor_msgs::Imu>();
            if (imu_msg != nullptr) {
                // 写入文件
                outfile << imu_msg->header.stamp.toSec() << " "
                        << imu_msg->linear_acceleration.x << " "
                        << imu_msg->linear_acceleration.y << " "
                        << imu_msg->linear_acceleration.z << " "
                        << imu_msg->angular_velocity.x << " "
                        << imu_msg->angular_velocity.y << " "
                        << imu_msg->angular_velocity.z << " "
                        << imu_msg->orientation.x << " "
                        << imu_msg->orientation.y << " "
                        << imu_msg->orientation.z << " "
                        << imu_msg->orientation.w << std::endl;
                if (view.size() <= 10) { 
                    std::cout << "Timestamp: " << imu_msg->header.stamp << std::endl;
                    std::cout << "Linear Accel: " 
                              << imu_msg->linear_acceleration.x << ", "
                              << imu_msg->linear_acceleration.y << ", "
                              << imu_msg->linear_acceleration.z << std::endl;
                }
            }
        }
        outfile.close();
        bag.close();

        std::cout << "Data saved successfully to " << output_file << std::endl;
    } catch (const rosbag::BagException& e) {
        std::cerr << "Error opening bag file: " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}