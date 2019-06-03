#include <cstdio>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/char.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "tello_msgs/srv/tello_action.hpp"
#include "tello_msgs/msg/tello_response.hpp"

#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

class TelloController : public rclcpp::Node
{

public:
    TelloController()
        : Node("tell_controller_adyien")
        , default_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default))
    {
        //initialize message and other primitive private fields
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
        sub_key_input = create_subscription<std_msgs::msg::Char>("/raw_keyboard", default_qos, std::bind(&TelloController::sub_key_input_callback, this, std::placeholders::_1));
        sub_tello_response = create_subscription<tello_msgs::msg::TelloResponse>("/solo/tello_response", default_qos, std::bind(&TelloController::sub_tello_response_callback, this, std::placeholders::_1));
        sub_demo_position = create_subscription<geometry_msgs::msg::Point>("/solo/demo_position", default_qos, std::bind(&TelloController::sub_demo_position_callback, this, std::placeholders::_1));

        //creat publishers
        pub_tello_twist = create_publisher<geometry_msgs::msg::Twist>("/solo/cmd_vel", default_qos);

        //create clock (for tf2 buffer)
        clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

        //creat TF2
        tf2_buffer = std::make_shared<tf2_ros::Buffer>(clock);
        tf2_listener = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer);
        tf2_valid = false;
        last_time_tf2_valid = rclcpp::Time(0,0);

        //define demo station point
        demo_position = tf2::Transform();
        demo_position.setOrigin(tf2::Vector3(0.0,0.2,1.0));
        demo_position_angle_offset = 0.2;
    }

private:

    //Quality of Service
    rclcpp::QoS default_qos;

    //ROS2 timer
    rclcpp::TimerBase::SharedPtr                                    tick_timer;

    //ROS2 communication
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr         srv_tello_cmd;
    rclcpp::Subscription<std_msgs::msg::Char>::SharedPtr            sub_key_input;
    rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr sub_tello_response;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr      sub_demo_position;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr         pub_tello_twist;
    rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture      future_srv_tello_cmd;

    //storage of received messages fields
    size_t tick_counter;
    char key_input;
    char Tello_sub_rc;
    char tello_srv_rc;
    std::string Tello_sub_string;

    //ROS2 clock
    rclcpp::Clock::SharedPtr clock;

    //ROS2 TF2
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener;
    geometry_msgs::msg::Transform DroneInMarker001Frame;
    bool tf2_valid;
    rclcpp::Time last_time_tf2_valid;

    //positioning
    tf2::Transform demo_position;
    double demo_position_angle_offset;

    /************************\
    |    MEMBER FUNCTIONS    |
    \************************/

    //client send message to drone with attached callback
    void send_tello_cmd (std::string cmd){
        while (!srv_tello_cmd->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
              RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
              return;
            }
            RCLCPP_INFO(this->get_logger(), "waiting for sertf2 ros needs both frames to be publishedvice to appear...");
        }
        auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
        request->set__cmd(cmd);
        future_srv_tello_cmd = srv_tello_cmd->async_send_request(request, std::bind(&TelloController::srv_tello_cmd_callback, this, std::placeholders::_1));
    }

    //callbacks of subscriptions and services
    void sub_key_input_callback(const std_msgs::msg::Char::SharedPtr msg){
        key_input = msg->data;
    }

    void sub_tello_response_callback(const tello_msgs::msg::TelloResponse::SharedPtr msg){
        Tello_sub_rc = msg->rc;
        Tello_sub_string = msg->str.c_str();
    }

    void srv_tello_cmd_callback(rclcpp::Client<tello_msgs::srv::TelloAction>::SharedFuture future){
        tello_srv_rc = future.get()->rc;
    }

    void sub_demo_position_callback(const geometry_msgs::msg::Point::SharedPtr msg){
        demo_position.setOrigin(tf2::Vector3(msg.get()->x,msg.get()->y,msg.get()->z));
    }

    //definition state mashine abstract class inferited by all states
    class SuperState {
       public:
        virtual SuperState* next_state(TelloController*) = 0;
    };
    SuperState* tello_state;

    // logic loop: update state mashine and reset inputs
    void tick_timer_callback(){
        RCLCPP_INFO(this->get_logger(), "\n  Timer:%i\tkey: %c  ", tick_counter, key_input);
        std::printf("  srv_rc: %i\t\t  sub_rc: %i\tsub_str: %s\n", tello_srv_rc, Tello_sub_rc, Tello_sub_string.c_str());
        if(tf2_buffer->canTransform("marker_001", tf2::TimePointZero, "base_link", tf2::TimePointZero, "map")){
            geometry_msgs::msg::Transform sample = tf2_buffer->lookupTransform("marker_001", tf2::TimePointZero, "base_link", tf2::TimePointZero, "map").transform;
            tf2_valid = sample != DroneInMarker001Frame;
            if(tf2_valid){
            DroneInMarker001Frame = sample;
            last_time_tf2_valid  = this->now(); //lookupTransform timestamp is buggy
            }
        }else{
            tf2_valid = false;
        }
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
        std::printf("\n");
    }

    /***********************\
    | STATE MASHINE CLASSES |
    \***********************/

    class StateRest : public SuperState {
        SuperState*  next_state(TelloController* node) override{
            std::printf("  state: Rest\n");
            if (node->key_input == 'q'){
                return new StateTakeOff;
            }
            return this;
        }
    };

    class StateTakeOff : public SuperState {
        SuperState*  next_state(TelloController* node) override{
            std::printf("  state: takeoff\n");
            if((node->tello_srv_rc == tello_msgs::srv::TelloAction::Response::ERROR_BUSY && node->Tello_sub_rc == tello_msgs::msg::TelloResponse::OK   )
            || (node->tello_srv_rc == tello_msgs::srv::TelloAction::Response::OK         && node->Tello_sub_rc != 0)){
                return new StateSteady;
            }
            node->send_tello_cmd("takeoff");
            return this;
        }
    };

    class StateSteady : public SuperState{
        SuperState*  next_state(TelloController* node) override{
            std::printf("  state: Steady\n");
            if (node->key_input == 'q'){
                return new StateLand();
            }else if(node->key_input == 'w'){
                return new StateSearchAruco(node);
            }
            return this;
        }
    };

    class StateLand : public SuperState {
        SuperState*  next_state(TelloController* node) override{
            std::printf("  state: Land\n");
            if((node->tello_srv_rc == tello_msgs::srv::TelloAction::Response::ERROR_BUSY && node->Tello_sub_rc == tello_msgs::msg::TelloResponse::OK   )
            || (node->tello_srv_rc == tello_msgs::srv::TelloAction::Response::OK         && node->Tello_sub_rc != 0)){
                return new StateRest;
            }
            node->send_tello_cmd("land");
            return this;
        }
    };

    class StateSearchAruco : public SuperState {
    public:
        rclcpp::Time stop;
        geometry_msgs::msg::Twist twist;
        StateSearchAruco(TelloController* node){
            stop = node->now() + rclcpp::Duration(20,0);
            twist = geometry_msgs::msg::Twist();
            twist.linear.z = 0.20;
            twist.angular.z = 0.6;
        }

        SuperState*  next_state(TelloController* node) override{
            std::printf("  state: SearchAruco\n");

            if (node->key_input == 'q'){
                node->pub_tello_twist->publish(geometry_msgs::msg::Twist());
                return new StateSteady;
            }else if(node->tf2_valid){
                node->pub_tello_twist->publish(geometry_msgs::msg::Twist());
                return new StateGoToPosition();
            }else if(node->now() < stop){
                node->pub_tello_twist->publish(this->twist);
                return this;
            }
            node->pub_tello_twist->publish(geometry_msgs::msg::Twist());
            return new StateSteady;
        }
    };

    class StateGoToPosition : public SuperState {

        SuperState*  next_state(TelloController* node) override{
            tf2::Vector3 Kp = tf2::Vector3(0.5,0.5,0.5);
            tf2::Vector3 Ki = tf2::Vector3(0.3,0.3,0.2);
            tf2::Vector3 integral = tf2::Vector3(0.0,0.0,0.0);
            double dt = 0.2;
            std::printf("  state: GoToPosition\n");
            if (node->key_input == 'q'){
                node->pub_tello_twist->publish(geometry_msgs::msg::Twist());
                return new StateSteady;
            }else if(node->tf2_valid){
                #define Drone_pos_t node->DroneInMarker001Frame.translation
                #define Drone_pos_r node->DroneInMarker001Frame.rotation
                tf2::Quaternion transform_quaternion = tf2::Quaternion(Drone_pos_r.x, Drone_pos_r.y, Drone_pos_r.z, Drone_pos_r.w);
                tf2::Vector3 transform_translation = tf2::Vector3(Drone_pos_t.x, Drone_pos_t.y, Drone_pos_t.z);
                tf2::Vector3 marker_ref_drone_tran = node->demo_position.getOrigin() - transform_translation;
                tf2::Vector3 drone_ref_drone_tran = tf2::quatRotate(transform_quaternion.inverse(), marker_ref_drone_tran );
                tf2::Vector3 translation_drone_to_marker = tf2::quatRotate(transform_quaternion.inverse(), transform_translation);
                #define error drone_ref_drone_tran
                bool in_position = (error.x() < 0.20 && error.y() < 0.20 && error.z() < 0.15);
                tf2::Vector3 Pout = error*Kp;
                if(in_position){
                    integral += error * dt;
                }else{
                    integral = tf2::Vector3(0.0,0.0,0.0);
                }
                tf2::Vector3 Iout = integral * Ki;
                tf2::Vector3 output = Pout + Iout;
                double x = std::max(std::min(output.x(),0.3),-0.3);
                double y = std::max(std::min(output.y(),0.3),-0.3);
                double z = std::max(std::min(output.z(),0.3),-0.3);

                double angle = std::atan(translation_drone_to_marker.y()/translation_drone_to_marker.x());
                double future_angle = std::atan((translation_drone_to_marker.y()+0.6*y)/(translation_drone_to_marker.x()+0.6*x));
                double a;
                if((std::abs(future_angle) < std::abs(angle)) && !in_position){
                    a = 0;
                } else {
                    a = in_position? 1.2 * (future_angle + node->demo_position_angle_offset) : 2.0 * future_angle;
                }
                double sin_a = std::sin(-a);
                double cos_a = std::cos(-a);
                double x_new = cos_a * x - sin_a * y;
                y = sin_a * x + cos_a * y;
                x  = x_new;
                std::printf("  positions\tx: %lf\ty: %lf\tz: %lf\n", Drone_pos_t.x, Drone_pos_t.y, Drone_pos_t.z);
                std::printf("  vector   \tx: %lf\ty: %lf\tz: %lf\ta: %lf\n", x, y, z, a);
                std::printf("  integral \tx: %lf\ty: %lf\tz: %lf\n", integral.x(), integral.y(), integral.z());
                geometry_msgs::msg::Twist twist = geometry_msgs::msg::Twist();
                twist.angular.z = a;
                twist.linear.x = x>0.01 ? x+0.02 : x<-0.01? x-0.02 : x;
                twist.linear.y = y>0.01 ? y+0.02 : y<-0.01? y-0.02 : y;
                twist.linear.z = z>0.01 ? z+0.05 : z<-0.01? z-0.05 : 0;
                node->pub_tello_twist->publish(twist);
                return this;
            }
            node->pub_tello_twist->publish(geometry_msgs::msg::Twist());
            if(node->last_time_tf2_valid + rclcpp::Duration(5,0) > node->now()){
                return this;
            }
            return new StateSteady();
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
