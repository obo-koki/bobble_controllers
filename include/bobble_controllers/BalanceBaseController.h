/******************************************************************************
 * Document this if it actually works...
 * Mike
*******************************************************************************/

#ifndef BOBBLE_CONTROLLERS_BALANCE_CONTROLLER_H
#define BOBBLE_CONTROLLERS_BALANCE_CONTROLLER_H

#include <cstddef>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <thread>
#include <mutex>

#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <geometry_msgs/Twist.h>
#include <bobble_controllers/BalanceControllerData.h>
//#include <bobble_controllers/gainConfig.h>
//#include <dynamic_reconfigure/server.h>

namespace bobble_controllers {

    class BalanceBaseController {
    public:
        BalanceBaseController(void){};
        ~BalanceBaseController(void);
        void init(ros::NodeHandle& nh);
        void reset();
        void update();
        void runSubscriber();
        int getControlMode() { return state.ActiveControlMode; };

    protected:
        ros::NodeHandle node_;
        bool run_thread_;
        realtime_tools::RealtimePublisher<bobble_controllers::BobbleBotStatus>* pub_bobble_status_;
        ros::Subscriber sub_command_;
        std::mutex control_command_mutex_;
	    std::thread* subscriber_thread_;
        bobble_controllers::BalanceControllerConfig   config;
        bobble_controllers::BalanceControllerCommands received_commands;
        bobble_controllers::BalanceControllerCommands processed_commands;
        bobble_controllers::BalanceControllerState    state;
        bobble_controllers::BalanceControllerOutputs  outputs;
        bobble_controllers::BalanceControllerFilters  filters;
        bobble_controllers::BalancePIDControllers     pid_controllers;
        virtual void estimateState() = 0;
        virtual void sendMotorCommands() = 0;
        virtual void loadConfig();
        void setupFilters();
        void setupControllers();
        void runStateLogic();
        void subscriberCallBack(const bobble_controllers::ControlCommands::ConstPtr &cmd);
        void cmdVelCallback(const geometry_msgs::Twist& command);
        void idleMode();
        void diagnosticMode();
        void startupMode();
        void balanceMode();
        void unpackParameter(std::string parameterName, double &referenceToParameter, double defaultValue);
        void unpackParameter(std::string parameterName, std::string &referenceToParameter, std::string defaultValue);
        void unpackFlag(std::string parameterName, bool &referenceToFlag, bool defaultValue);
        double limit(double cmd, double max);

        //dynamic_reconfigure::Server<bobble_controllers::gainConfig> param_server_;
        //dynamic_reconfigure::Server<bobble_controllers::gainConfig>::CallbackType callback_;
        //void param_callback(const bobble_controllers::gainConfig& config, uint32_t level);

    private:
        void populateImuData();
        void clearCommandState(bobble_controllers::BalanceControllerCommands& cmds);
        void populateCommands();
        void write_controller_status_msg();
        void applyFilters();
        void applySafety();
    };
}
#endif
