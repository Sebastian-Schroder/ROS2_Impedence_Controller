
#include "impedence_controller/impedence_controller.hpp"
#include "impedence_controller/visability_control.h"
#include "pinocchio/parsers/urdf.hpp"
//TODO https://github.com/ros-controls/ros2_controllers/blob/master/doc/writing_new_controller.rst
using namespace std::chrono_literals;
namespace pin = pinocchio;

namespace impedence_controller{



ImpedenceController::ImpedenceController(/* args */)
: controller_interface::ControllerInterface() 
{
    this->dataPtr->parameter_client = std::make_shared<rclcpp::SyncParametersClient>(get_node(),"/robot_state_publisher");
    while(!this->dataPtr->parameter_client->wait_for_service(1s)){
        if(!rclcpp::ok()){
            RCLCPP_ERROR(get_node()->get_logger(),"Interrupted, Exiting");
            rclcpp::shutdown();
        }
        RCLCPP_INFO(get_node()->get_logger(),"wating for /robot_state_publisher");
    }
    
 
    

    
    

    
    equilibrium ;

}

ImpedenceController::~ImpedenceController()
{
}



    controller_interface::InterfaceConfiguration ImpedenceController::command_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        if(dof==0){
            RCLCPP_ERROR(get_node()->get_logger(),"During ros2_control interface configuration, degrees of freedom [%u] is not valid;",dof);
            rclcpp::shutdown();
        }
        // conf.names.reserve(dof*)


    }

    controller_interface::InterfaceConfiguration ImpedenceController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration conf;
        conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    }


    controller_interface::return_type ImpedenceController::update(const rclcpp::Time & time, const rclcpp::Duration & period){
        if(get_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
            return(controller_interface::return_type::OK);
        }

        //? update trajectory;

        //? calculate necessary torques;
        

        //? write commands


        //? publish messages

        publish_state(time);
        
    }



    controller_interface::CallbackReturn ImpedenceController::on_init(){
        auto parameters = this->dataPtr->parameter_client->get_parameters({ "robot_description" });
        std::string urdf_string;
        for (auto& parameter : parameters)
        {
            if (parameter.get_name() == "robot_description")
            {
                urdf_string = parameter.value_to_string();
                break;
            }
        }
        _model = std::make_unique<pin::Model>();
        pinocchio::urdf::buildModelFromXML(urdf_string,*_model);
        // _model->loadFromString(urdf_string); // idk why load from string doesn't build correctly.
        _model_data = std::make_unique<pin::Data>(*_model);
        RCLCPP_INFO(get_node()->get_logger(),"URDF loaded with name [%s]",_model->name.c_str());

        rclcpp::QoS qos(10);
        qos.reliable().transient_local();
        // initialise the realtime publisher.
        this->dataPtr->realtime_wrapped_interface = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/impedence_controller",qos);
        this->dataPtr->torque_publisher = std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(this->dataPtr->realtime_wrapped_interface);
        
        
    }

    controller_interface::CallbackReturn ImpedenceController::on_activate(const rclcpp_lifecycle::State & previous_state){}



    controller_interface::CallbackReturn ImpedenceController::on_configure(const rclcpp_lifecycle::State & previous_state){}

    controller_interface::CallbackReturn ImpedenceController::on_deactivate(const rclcpp_lifecycle::State & previous_state){}

    controller_interface::CallbackReturn ImpedenceController::on_cleanup(const rclcpp_lifecycle::State & previous_state){}
    controller_interface::CallbackReturn ImpedenceController::on_error(const rclcpp_lifecycle::State & previous_state){}

    controller_interface::CallbackReturn ImpedenceController::on_shutdown(const rclcpp_lifecycle::State & previous_state){}
    
    void ImpedenceController::publish_state(const rclcpp::Time & time){
        // try to get lock for publisher;
        bool lock_is_held = this->dataPtr->torque_publisher->trylock();
        for(int i = 0; i < 10 && !lock_is_held; i++){
            lock_is_held=this->dataPtr->torque_publisher->trylock();
            std::this_thread::sleep_for(100ms);

        }
        if(!lock_is_held){
            RCLCPP_WARN(get_node()->get_logger(),"failed to get realtime publisher lock");
            return;
        }

        auto & msg = dataPtr->torque_publisher->msg_;
        //TODO write the message and publish it.
        
        for(int i=0; i < this->dof;i++){
            // msg.data[i];
            // this->dataPtr->active_goal.readFromRT()
        }
        

        dataPtr->torque_publisher->unlockAndPublish(); // send the message
    }

    std::array<double,7> ImpedenceController::calculate_torques(){

        // auto coriolis = _model;
        Eigen::Matrix<double,7,1> q, dq; //v
        pinocchio::forwardKinematics(*_model,*_model_data,q,dq);

        auto end_effector = _model_data->oMi.back();

        Eigen::Vector3d position(end_effector.translation());
        Eigen::Quaterniond rotation(end_effector.rotation());


        Eigen::Vector<double,6> error;
        error.head(3) << position- equilibrium.translation();
        
        if( Eigen::Quaterniond(equilibrium.rotation()).coeffs().dot(rotation.coeffs())<0){
            rotation.coeffs() << -rotation.coeffs();
        }
        // spring damper system
         Eigen::Quaterniond error_quaternion(rotation.inverse() * Eigen::Quaterniond(equilibrium.rotation()));
         error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
         error.tail(3) << -end_effector.rotation()* error.tail(3);


        Eigen::VectorXd torques(dof),desired_torques(dof);

        torques << _model_data->J.transpose()*(-stiffness*error-damping*(_model_data->J*dq));
        desired_torques = torques + _model_data->C;

        std::array<double,7> desired_torque_array;
        Eigen::VectorXd::Map(&desired_torque_array[0],7) = desired_torques;

        return(desired_torque_array);



    }


};

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    impedence_controller::ImpedenceController, controller_interface::ControllerInterface
)