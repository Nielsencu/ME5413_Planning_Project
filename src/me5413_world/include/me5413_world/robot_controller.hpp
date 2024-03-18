#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <memory>

#include "me5413_world/base_controller.hpp"
#include "me5413_world/dwa.hpp"
#include "me5413_world/pure_pursuit.hpp"
#include "me5413_world/pid.hpp"

namespace control{

    enum class LinearController{PID, DWA};
    enum class AngularController{PID, PURE_PURSUIT, STANLEY, DWA};

    class RobotController{
    public:
        RobotController(LinearController linControllerIn, AngularController angControllerIn);
        LinearController _linearControllerType;
        AngularController _angularControllerType;
        BaseController* getLinearController(){return _linearController;};
        BaseController* getAngularController(){return _angularController;};
        std::array<double,2> getLinXAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal);
    private:
        BaseController* _linearController;
        BaseController* _angularController;
        std::function<double (const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)> _velXFn;
        std::function<double (const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)> _angZFn;
    };

    RobotController::RobotController(LinearController linControllerIn, AngularController angControllerIn){
        _linearControllerType = linControllerIn;
        _angularControllerType = angControllerIn;
        if(linControllerIn == LinearController::DWA){
            _linearController = dynamic_cast<BaseController*>(new DWAController());
        }else{
            _linearController = dynamic_cast<BaseController*>(new PIDController());
            _velXFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _linearController->getLinX(odom_robot, pose_goal);};
        }
        if(angControllerIn == AngularController::PURE_PURSUIT){
            _angularController = dynamic_cast<BaseController*>(new PurePursuitController(false));
            _angZFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _angularController->getAngZ(odom_robot, pose_goal);};
        }else if(angControllerIn == AngularController::STANLEY){
            _angularController = dynamic_cast<BaseController*>(new PurePursuitController(true));
            _angZFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _angularController->getAngZ(odom_robot, pose_goal);};
        }else{
            _angularController = dynamic_cast<BaseController*>(new PIDController());
            _angZFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _angularController->getAngZ(odom_robot, pose_goal);};
        }
    };
    
    std::array<double,2> RobotController::getLinXAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
        // if(_linearControllerType == LinearController::DWA){
        //     return _linearController->getControl(odom_robot, pose_goal);
        // }
        return std::array<double, 2>{_velXFn(odom_robot, pose_goal), _angZFn(odom_robot, pose_goal)};
    }
}