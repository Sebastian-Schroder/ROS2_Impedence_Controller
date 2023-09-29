#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <controller_interface/controller_interface.hpp>
#include <vector>
#include <string>
#ifndef IMPEDENCE_CONTROLLER
#define IMPEDENCE_CONTROLLER


namespace impedence_controller{

//TODO https://github.com/ros-controls/ros2_controllers/blob/master/doc/writing_new_controller.rst


class ImpedenceControllerPrivate {
    public:
        std::vector<std::string> joint_names;
        std::vector<stdc::string> interface_names;
};

class ImpedenceController: public controller_interface::ControllerInterface
{
private:
    /* data */
public:
    ImpedenceController(/* args */);
    ~ImpedenceController();


    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;


    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;



    controller_interface::CallbackReturn on_init(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;



    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

};






};




#endif