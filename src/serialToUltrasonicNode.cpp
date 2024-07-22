#include <cstdio>
#include <iostream>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "serial/serial.h" // Make sure to install the 'serial' package

using std::placeholders::_1;

class SerialUltrasonicNode : public rclcpp::Node
{
public:
    SerialUltrasonicNode() : Node("serial_ultrasonic_node")
    {
        this->declare_parameter("verbose", true);
        this->print_publish = this->get_parameter("verbose").as_bool();

        // Initialize serial port
        serial_port_.setPort("/dev/ttyACM0"); // Change port as required
        serial_port_.setBaudrate(115200); // Change baudrate as required
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(timeout);
        serial_port_.open();

        // Publisher for the ROS topic
        publisher_left_ = this->create_publisher<sensor_msgs::msg::Range>("range_left", 10);
        publisher_right_ = this->create_publisher<sensor_msgs::msg::Range>("range_right", 10);

        // Timer to read from serial port and publish data
        timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&SerialUltrasonicNode::serial_to_ros, this));
    }

private:
    void serial_to_ros()
    {
        bool serial_available = false;
        try{
            serial_available = serial_port_.available();
        }
        catch(const serial::IOException& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Serial IO Exception: %s", e.what());
            while (1)
            {
                // Attempt to close and reopen the connection
                try {
                    serial_port_.close();
                    serial_port_.open();
                    RCLCPP_INFO(this->get_logger(), "Reconnected to Ultrasonic");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to reconnect: %s", e.what());
                    rclcpp::sleep_for(std::chrono::milliseconds(500));// Wait before retrying
                    continue;
                }
                break;
            }
        }
        
        if (serial_available > 0)
        {
            this->inactive_counter = 0;
            // Read data from serial port
            std::string serial_data;
            try{
                 serial_data = serial_port_.readline();
            }
            catch(const serial::IOException& e){
                //RCLCPP_INFO(this->get_logger(), "Serial connection broken");
                //serial_data= "broken serial connection";
                RCLCPP_ERROR(this->get_logger(), "Serial IO Exception: %s", e.what());
                while (1)
                {
                    // Attempt to close and reopen the connection
                    try {
                        serial_port_.close();
                        serial_port_.open();
                        RCLCPP_INFO(this->get_logger(), "Reconnected to Ultrasonic");
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to reconnect: %s", e.what());
                        // Wait before retrying
                        rclcpp::sleep_for(std::chrono::milliseconds(500));
                        continue;
                    }
                    break;                        
                }
                return;
            }

            if (isalpha(serial_data[0])){ 
                RCLCPP_INFO(this->get_logger(), "No-data Range message");
                std::cout << serial_data << std::endl;
                return; 
            }

            sensor_msgs::msg::Range range_l_msg;
            sensor_msgs::msg::Range range_r_msg;
            
            bool range_msg_valid = parse_range_data(range_l_msg, range_r_msg, serial_data);

            if (range_msg_valid)
            {
                // Publish the Range data to ROS topic
                publisher_left_->publish(range_l_msg);
                publisher_right_->publish(range_r_msg);

                // Log the published data
                if (this->print_publish)
                {
                    RCLCPP_INFO(this->get_logger(), "Published Range data");
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Invalid input Range data");
                std::cout << serial_data << std::endl;
            }
        }
        else
        {
            this->inactive_counter++;
            if (this->inactive_counter > 1000)
            {
                this->inactive_counter = 0;
                RCLCPP_INFO(this->get_logger(), "No Range data received in last 5 seconds");
            }
        }
    }

    bool verifyChecksum(const std::string& message, std::string delimiter) {
        size_t last_delim = message.find_last_of(delimiter);
        if (last_delim == std::string::npos) {
            return false;
        }

        std::string data = message.substr(0, last_delim);
        int received_checksum = stoi(message.substr(last_delim + delimiter.length()));
        int calculated_checksum = 0;

        for (char c : data) {
            calculated_checksum += c;
        }

        return calculated_checksum == received_checksum;
    }

    bool parse_range_data(sensor_msgs::msg::Range &range_l_msg, 
                          sensor_msgs::msg::Range &range_r_msg, 
                          const std::string &serial_data)
    {
        std::string delimiter = ",";
        if (!verifyChecksum(serial_data, delimiter)) 
        {
            RCLCPP_INFO(this->get_logger(), "Invalid checksum");
            return false;
        }

        std::vector<double> values;
        std::stringstream ss(serial_data);
        //int i = 0;
        double value;
        while (ss >> value){
            
            values.push_back(value);

            char nextchar = ss.peek();
            if (nextchar == '\t'){
                ss.ignore();
            }
            if (nextchar == ','){
                ss.ignore();
            }
            if (nextchar == '\n'){
                break;
            }
        }

        // left_val, right_val, checksum
        if (values.size() != 3){ 
            return false; // invalid incoming message
        }

        float field_of_view = 0.87;  // 50 degrees in radians
        float min_range = 0.2;  // Minimum range in meters
        float max_range = 6.0;  // Maximum range in meters
        float msg_time_stamp = this->get_clock()->now();

        // Set the fields of the Range message
        range_l_msg.header.stamp = msg_time_stamp;
        range_l_msg.header.frame_id = "range_left_link";
        range_l_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_l_msg.field_of_view = field_of_view;  
        range_l_msg.min_range = min_range;  
        range_l_msg.max_range = max_range;  
        range_l_msg.range = values[0]; 

        range_r_msg.header.stamp = msg_time_stamp;
        range_r_msg.header.frame_id = "range_right_link";
        range_r_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_r_msg.field_of_view = field_of_view;  
        range_r_msg.min_range = min_range;  
        range_r_msg.max_range = max_range;  
        range_r_msg.range = values[1];

        return true; // valid incoming message
    }

    bool print_publish = true;
    int inactive_counter = 0;
    serial::Serial serial_port_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_left_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher_right_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<SerialUltrasonicNode>();
        rclcpp::spin(node);
    }
    catch(const serial::IOException& e)
    {
        std::cerr << "CRASHED IN MAIN\n";
        std::cerr << e.what() << '\n';
    }
    
    rclcpp::shutdown();
    return 0;
}
