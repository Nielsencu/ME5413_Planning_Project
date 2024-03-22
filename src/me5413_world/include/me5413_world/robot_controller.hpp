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
        std::shared_ptr<PIDController> getPIDController(){return _pidController;};
        std::shared_ptr<StanleyController> getStanleyController(){return _stanleyController;};
        std::array<double,2> getLinXAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal);
    private:
        // BaseController* _linearController;
        // BaseController* _angularController;

        std::shared_ptr<PurePursuitController> _purePursuitController;
        std::shared_ptr<PIDController> _pidController;
        std::shared_ptr<DWAController> _dwaController;
        std::shared_ptr<StanleyController> _stanleyController;

        std::function<double (const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)> _velXFn;
        std::function<double (const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal)> _angZFn;
    };

    RobotController::RobotController(LinearController linControllerIn, AngularController angControllerIn){
        _linearControllerType = linControllerIn;
        _angularControllerType = angControllerIn;
        if(linControllerIn == LinearController::DWA){
            _dwaController = std::make_shared<DWAController>();
            return;
        }else{
            _pidController = std::make_shared<PIDController>();
            _velXFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _pidController->getLinX(odom_robot, pose_goal);};
        }
        if(angControllerIn == AngularController::PURE_PURSUIT){
            _purePursuitController = std::make_shared<PurePursuitController>();
            _angZFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _purePursuitController->getAngZ(odom_robot, pose_goal);};
        }else if(angControllerIn == AngularController::STANLEY){
            _stanleyController = std::make_shared<StanleyController>();
            _angZFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _stanleyController->getAngZ(odom_robot, pose_goal);};
        }else{
            _pidController = std::make_shared<PIDController>();
            _angZFn = [this](const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){return _pidController->getAngZ(odom_robot, pose_goal);};
        }
    };
    
    std::array<double,2> RobotController::getLinXAngZ(const nav_msgs::Odometry& odom_robot, const geometry_msgs::Pose& pose_goal){
        if(_linearControllerType == LinearController::DWA && _dwaController){
            return _dwaController->getControl(odom_robot, pose_goal);
        }
        return std::array<double, 2>{_velXFn(odom_robot, pose_goal), _angZFn(odom_robot, pose_goal)};
    }
}