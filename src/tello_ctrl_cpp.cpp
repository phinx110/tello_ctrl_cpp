#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/char.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "tello_msgs/srv/tello_action.hpp"
#include "tello_msgs/msg/tello_response.hpp"



using namespace std::chrono_literals;


class TelloController : public rclcpp::Node
{

    constexpr static const auto timer_periode = 500ms;


public:
    TelloController() : Node("tell_controller_adyien"), tick_counter(0)
    {
        //create timers
        tick_timer = create_wall_timer(500ms, std::bind(&TelloController::tick_timer_callback, this));

        //creat actions
        srv_tello_cmd = create_client<tello_msgs::srv::TelloAction>("/solo/tello_action");

        //creat subscriptions
        sub_key_input = create_subscription<std_msgs::msg::Char>("/raw_keyboard", std::bind(&TelloController::key_input_callback, this, std::placeholders::_1));
        sub_tello_response = create_subscription<tello_msgs::msg::TelloResponse>("/solo/tello_response", std::bind(&TelloController::tello_response_callback, this, std::placeholders::_1));

        //creat publishers
        pub_tello_twist = create_publisher<geometry_msgs::msg::Twist>("/solo/cmd_vel");


    }


private:
    rclcpp::TimerBase::SharedPtr                                    tick_timer;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr         srv_tello_cmd;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr            sub_key_input;
    rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr sub_tello_response;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_tello_twist;

    size_t tick_counter;
    char key_input;
    u_char Tello_sub_rc;
    char tello_srv_rc;
    std::string Tello_sub_string;



    void tick_timer_callback(){
        tick_counter++;
        RCLCPP_INFO(this->get_logger(), "Timer:%i", tick_counter);
    }

    void key_input_callback(const std_msgs::msg::Char::SharedPtr msg){
        key_input = msg->data;
        RCLCPP_INFO(this->get_logger(), "Key: %c", msg->data);
    }

    void tello_response_callback(const tello_msgs::msg::TelloResponse::SharedPtr msg){
        Tello_sub_rc = msg->rc;
        Tello_sub_string = msg->str.c_str();
        RCLCPP_INFO(this->get_logger(), "rc: %c,\tString: %s", msg->rc,msg->str.c_str());
    }


};

/*
class TelloStateInt {

public:
    TelloStateInt next_state(char key_input);
};

class state_rest : TelloStateInt{
    state_rest next_state(char key_input){
        if( key_input == 'q'){
            return new state_liftoff();
        }
    }
};

class state_liftoff : TelloStateInt{
    state_liftoff next_state(char key_input){
        return this;
    }
};

class state_steady : TelloStateInt{
    TelloStateInt next_state(char key_input);
};

class state_land : TelloStateInt{
    TelloStateInt next_state(char key_input);
};


*/

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto drone1 = std::make_shared<TelloController>();


    rclcpp::spin(drone1);
    rclcpp::shutdown();
    return 0;
}
