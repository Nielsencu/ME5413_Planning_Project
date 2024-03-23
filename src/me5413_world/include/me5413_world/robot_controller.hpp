#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <memory>

#include "me5413_world/base_controller.hpp"
#include "me5413_world/dwa.hpp"
#include "me5413_world/pure_pursuit.hpp"
#include "me5413_world/pid.hpp"

namespace control{
    enum class ControllerType{PID, PURE_PURSUIT, STANLEY, DWA};
    class RobotController{
    public:
        RobotController(ControllerType controllerTypeIn);
        ControllerType _controllerType;
        CmdVel getCmdVel(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal);
        std::shared_ptr<BaseController> getController(){return _controller;};
        void updateParams(control::PIDController::Params&& paramsIn);
        void updateParams(control::PurePursuitController::Params&& paramsIn);
        void updateParams(control::StanleyController::Params&& paramsIn);
    private:
        std::shared_ptr<BaseController> _controller;
    };

    RobotController::RobotController(ControllerType controllerTypeIn){
        _controllerType = std::move(controllerTypeIn);
        if(_controllerType == ControllerType::DWA){
            _controller = std::make_shared<DWAController>();
        }else if(_controllerType == ControllerType::PURE_PURSUIT){
            _controller = std::make_shared<PurePursuitController>();
        }else if(_controllerType == ControllerType::STANLEY){
            _controller = std::make_shared<StanleyController>();
        }else{
            _controller = std::make_shared<PIDController>();
        }
    };
    
    void RobotController::updateParams(control::PIDController::Params&& paramsIn){
        PIDController* pidController = nullptr;
        if(_controllerType == ControllerType::PURE_PURSUIT){
            const auto controller = dynamic_cast<PurePursuitController*>(_controller.get());
            pidController = controller->getPIDController();
        }else if(_controllerType == ControllerType::STANLEY){
            const auto controller = dynamic_cast<StanleyController*>(_controller.get());
            pidController = controller->getPIDController();
        }else if(_controllerType == ControllerType::PID){
            pidController = dynamic_cast<PIDController*>(_controller.get());
        }
        if(pidController != nullptr){
            pidController->updateParams(std::move(paramsIn));
        }
    }

    void RobotController::updateParams(control::PurePursuitController::Params&& paramsIn){
        const auto controller = dynamic_cast<PurePursuitController*>(_controller.get());
        if(controller != nullptr){
            controller->updateParams(std::move(paramsIn));
        }
    }

    void RobotController::updateParams(control::StanleyController::Params&& paramsIn){
        const auto controller = dynamic_cast<StanleyController*>(_controller.get());
        if(controller != nullptr){
            controller->updateParams(std::move(paramsIn));
        }
    }

    CmdVel RobotController::getCmdVel(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
        return _controller->getCmdVel(odom_robot, pose_goal);
    }
}