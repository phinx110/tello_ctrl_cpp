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

public:
    TelloController() : Node("tell_controller_adyien")
    {
        //initialize private fields
        tick_counter = 0;
        key_input = 0;
        Tello_sub_rc = 0;
        tello_srv_rc = 0;
        Tello_sub_string = "";

        //initialize private state machine
        tello_state =  new StateRest();

        //create timers
        tick_timer = create_wall_timer(200ms, std::bind(&TelloController::tick_timer_callback, this));

        //creat actions
        srv_tello_cmd = create_client<tello_msgs::srv::TelloAction>("/solo/tello_action");

        //creat subscriptions
        sub_key_input = create_subscription<std_msgs::msg::Char>("/raw_keyboard", std::bind(&TelloController::key_input_callback, this, std::placeholders::_1));
        sub_tello_response = create_subscription<tello_msgs::msg::TelloResponse>("/solo/tello_response", std::bind(&TelloController::tello_response_callback, this, std::placeholders::_1));

        //creat publishers
        pub_tello_twist = create_publisher<geometry_msgs::msg::Twist>("/solo/cmd_vel");

    }

private:

    //ROS2 stuff
    rclcpp::TimerBase::SharedPtr                                    tick_timer;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr         srv_tello_cmd;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr            sub_key_input;
    rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr sub_tello_response;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_tello_twist;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture      future_srv_tello_cmd;

    // storage of received messages
    size_t tick_counter;
    char key_input;
    char Tello_sub_rc;
    char tello_srv_rc;
    std::string Tello_sub_string;

    //client send message to drone with attached callback
    void send_tello_cmd (std::string cmd){
        while (!srv_tello_cmd->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
              return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
        }
        auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
        request->set__cmd(cmd);
        future_srv_tello_cmd = srv_tello_cmd->async_send_request(request, std::bind(&TelloController::srv_tello_cmd_callback, this, std::placeholders::_1));
    }

    //callbacks of subscriptions and services
    void key_input_callback(const std_msgs::msg::Char::SharedPtr msg){
        key_input = msg->data;
    }

    void tello_response_callback(const tello_msgs::msg::TelloResponse::SharedPtr msg){
        Tello_sub_rc = msg->rc;
        Tello_sub_string = msg->str.c_str();
    }

    void srv_tello_cmd_callback(rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture future){
        tello_srv_rc = future.get()->rc;
        RCLCPP_INFO(this->get_logger(), "Service: %c", tello_srv_rc);
    }

    //definition of parrent state mashine abstract class
    class SuperState {
       public:
        virtual SuperState* next_state(TelloController*) = 0;
    };
    SuperState* tello_state;

    // update state mashine and reset inputs
    void tick_timer_callback(){
        RCLCPP_INFO(this->get_logger(), "Timer:%i\tkey: %c\tsrv_rc: %c\tsub_rc: %c\tsub_str: %s", tick_counter, key_input, tello_srv_rc, Tello_sub_rc, Tello_sub_string.c_str());
        SuperState* sequal = tello_state->next_state(this);
        if(sequal!=tello_state){
            delete tello_state;
            tello_state = sequal;
        }

        key_input = 0;
        Tello_sub_rc = 0;
        tello_srv_rc = 0;
        Tello_sub_string = "";
        tick_counter++;
    }

    /***********************\
    | STATE MASHINE CLASSES |
    \***********************/

    class StateRest : public SuperState {
        SuperState*  next_state(TelloController* node) override{
            if (node->key_input == 'q' && node->tello_srv_rc == 0){
                return new StateTakeoff();
            }
            return this;
        }
    };

    class StateTakeoff : public SuperState {
        SuperState*  next_state(TelloController* node) override{
            if(node->tello_srv_rc == tello_msgs::srv::TelloAction::Response::OK){
                return new StateSteady;
            }
            node->send_tello_cmd("takeoff");
            return this;
        }
    };

    class StateSteady : public SuperState{
        SuperState*  next_state(TelloController* node) override{
            if (node->key_input == 'q' && node->tello_srv_rc == 0){
                return new StateLand();
            }
            return this;
        }
    };

    class StateLand : public SuperState {
        SuperState*  next_state(TelloController* node) override{
            if(node->tello_srv_rc == tello_msgs::srv::TelloAction::Response::OK){
                return new StateRest;
            }
            node->send_tello_cmd("land");
            return this;
        }
    };

    // end of TelloController class
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto drone1 = std::make_shared<TelloController>();
    rclcpp::spin(drone1);
    rclcpp::shutdown();
    return 0;
}
