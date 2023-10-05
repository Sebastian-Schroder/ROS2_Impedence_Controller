#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_interface/controller_interface.hpp>
#include <vector>
#include <string>

#include <pinocchio/parsers/sample-models.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/dynamics.hpp>

#include <eigen3/Eigen/Eigen>

#include <rcl_interfaces/msg/parameter_event.hpp>
#include "controller_interface/helpers.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include <std_msgs/msg/float64_multi_array.hpp>


#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_server_goal_handle.h>
#ifndef IMPEDENCE_CONTROLLER_HPP_
#define IMPEDENCE_CONTROLLER_HPP_



using namespace std::chrono_literals;
namespace impedence_controller{

//TODO https://github.com/ros-controls/ros2_controllers/blob/master/doc/writing_new_controller.rst


class ImpedenceControllerPrivate {
    public:
    realtime_tools::RealtimeBuffer<std::shared_ptr<realtime_tools::RealtimeServerGoalHandle<std_msgs::msg::Float64MultiArray>>> active_goal;
    std::shared_ptr<rclcpp::SyncParametersClient> parameter_client;
    realtime_tools::RealtimePublisherSharedPtr<std_msgs::msg::Float64MultiArray> torque_publisher;
    rclcpp::TimerBase::SharedPtr goal_handle_timer;
    rclcpp::Duration action_monitor_period = rclcpp::Duration(50ms);

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr realtime_wrapped_interface;
};


struct ImpedenceControllerParameters{
    std::vector<std::string> joints = {};
    std::vector<std::string> command_joints = {};
    std::vector<std::string> command_interfaces = {};
    std::vector<std::string> state_interfaces = {};
    
};

class ImpedenceController: public controller_interface::ControllerInterface
{

public:
    ImpedenceController(/* args */);
    ~ImpedenceController();


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;



    controller_interface::CallbackReturn on_init() override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;



    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;



    private:
        void publish_state(const rclcpp::Time & time);
        std::array<double,7> calculate_torques();
    /* data */

        
        // auto test = std::make_shared<ParamListener>(get_node());
        std::vector<std::string> joint_names;
        std::vector<std::string> interface_names;
        size_t dof{0};
        rclcpp::Parameter params;
    
    Eigen::Matrix<double,6,6> stiffness, damping; //TODO set these values from a yaml file


    std::unique_ptr<pinocchio::Model> _model;
    std::unique_ptr<pinocchio::Data> _model_data;
    
    Eigen::Affine3d equilibrium;

    std::unique_ptr<ImpedenceControllerPrivate> dataPtr;


};







};




#endif