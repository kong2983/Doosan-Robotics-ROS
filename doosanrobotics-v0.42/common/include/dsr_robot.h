/**    
    @file    dsr_robot.h
    @date    2018/11/23
    @author    kabdol2
    @brief    Robot Conrol Class
*/

#ifndef DSR_ROBOT_H
#define DSR_ROBOT_H

#include <ros/ros.h>
#include <signal.h>
#include <boost/thread/thread.hpp>
#include <cstdlib>
#include <array>
#include <vector>
#include <algorithm>
#include <string>
#include <iostream>

#include <dsr_msgs/RobotError.h>
#include <dsr_msgs/RobotState.h>
#include <dsr_msgs/RobotStop.h>

#include <dsr_msgs/MoveJoint.h>
#include <dsr_msgs/MoveLine.h>
#include <dsr_msgs/MoveJointx.h>
#include <dsr_msgs/MoveCircle.h>
#include <dsr_msgs/MoveSplineJoint.h>
#include <dsr_msgs/MoveSplineTask.h>
#include <dsr_msgs/MoveBlending.h>
#include <dsr_msgs/MoveSpiral.h>
#include <dsr_msgs/MovePeriodic.h>
#include <dsr_msgs/MoveWait.h>

#include <dsr_msgs/ConfigCreateTcp.h>
#include <dsr_msgs/ConfigDeleteTcp.h>
#include <dsr_msgs/GetCurrentTcp.h>
#include <dsr_msgs/SetCurrentTcp.h>

#include <dsr_msgs/SetCurrentTool.h>
#include <dsr_msgs/GetCurrentTool.h>
#include <dsr_msgs/ConfigCreateTool.h>
#include <dsr_msgs/ConfigDeleteTool.h>

#include <dsr_msgs/SetCtrlBoxDigitalOutput.h>
#include <dsr_msgs/GetCtrlBoxDigitalInput.h>
#include <dsr_msgs/SetToolDigitalOutput.h>
#include <dsr_msgs/GetToolDigitalInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutput.h>
#include <dsr_msgs/GetCtrlBoxAnalogInput.h>
#include <dsr_msgs/SetCtrlBoxAnalogOutputType.h>
#include <dsr_msgs/SetCtrlBoxAnalogInputType.h>

#include <dsr_msgs/SetModbusOutput.h>
#include <dsr_msgs/GetModbusInput.h>
#include <dsr_msgs/ConfigCreateModbus.h>
#include <dsr_msgs/ConfigDeleteModbus.h>

#include <dsr_msgs/DrlPause.h>
#include <dsr_msgs/DrlStart.h>
#include <dsr_msgs/DrlStop.h>
#include <dsr_msgs/DrlResume.h>

#include <dsr_msgs/GripperMove.h>

#include "DRFL.h"
#include "DRFC.h"
#include "DRFS.h"

using namespace std;

namespace DSR_Robot{
    class CDsrRobot
    {
        public:
            CDsrRobot(ros::NodeHandle nh, std::string robotID="dsr01", std::string robotModel="m1013");
            virtual ~CDsrRobot();

            int stop(int nMode = STOP_TYPE_QUICK);

            //----- sync motion
            int movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, 
                      int nMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE); 

            int movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, 
                      int   nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, 
                      int   nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);

            int movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, 
                       int nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0);

            int move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, 
                             int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL);

            int move_spiral(int nTaskAxis, float fRevolution, float fMaxRadius, float fMaxLength, 
                           float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, int nMoveReference = MOVE_REFERENCE_TOOL);

            int movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, 
                       int nMoveMode = MOVE_MODE_ABSOLUTE);

            int movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                       int nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);

            int moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                      int nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE);

            //----- async motion
            int amovej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, 
                      int nMoveMode = MOVE_MODE_ABSOLUTE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE); 

            int amovel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, 
                      int   nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);             

            int amovec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, 
                      int   nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE);

            int amovejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime=0.f, 
                       int nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, float fBlendingRadius = 0.f, int nBlendingType = BLENDING_SPEED_TYPE_DUPLICATE, int nSolSpace = 0);

            int amove_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime = 0.f, 
                             int nRepeat = 1, int nMoveReference = MOVE_REFERENCE_TOOL);

            int amove_spiral(int nTaskAxis, float fRevolution, float fMaxRadius, float fMaxLength, 
                           float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f, int nMoveReference = MOVE_REFERENCE_TOOL);

            int amovesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime = 0.f, 
                       int nMoveMode = MOVE_MODE_ABSOLUTE);

            int amovesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                       int nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE, int nVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT);

            int amoveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime = 0.f,
                      int nMoveMode = MOVE_MODE_ABSOLUTE, int nMoveReference = MOVE_REFERENCE_BASE);

            //----- Motion Wait

            int move_wait();

            //----- TCP
            int config_create_tcp(string strName, float fTargetPos[NUM_TASK]);
            int config_delete_tcp(string strName);
            int set_current_tcp(string strName);
            string get_current_tcp();

            //----- TOOL
            int config_create_tool(string strName, float fTargetWeight, float fTargetCog[3], float fTargetInertia[NUM_TASK]);
            int config_delete_tool(string strName);
            int set_current_tool(string strName);
            string get_current_tool();

            //----- IO
            int set_digital_output(int nGpioIndex, bool bGpioValue);
            int get_digital_input(int nGpioIndex);
            int set_tool_digital_output(int nGpioIndex, bool bGpioValue);
            int get_tool_digital_input(int nGpioIndex);
            int set_analog_output(int nGpioChannel, float fGpioValue);
            int get_analog_input(int nGpioChannel);
            int set_analog_output_type(int nGpioChannel, int nGpioMode);
            int set_analog_input_type(int nGpioChannel, int nGpioMode);

            //----- MODBUS
            int config_create_modbus(string strName, string strIP, int nPort, int nRegType, int nRegIndex, int nRegValue = 0);
            int config_delete_modbus(string strName);
            int set_modbus_output(string strName, int nValue);
            int get_modbus_input(string strName);

            //----- DRL        
            int drl_start(int nRobotSystem, string strCode);
            int drl_stop(int nStopMode = STOP_TYPE_QUICK);
            int drl_pause();
            int drl_resume();

        private:
            int _movej(float fTargetPos[NUM_JOINT], float fTargetVel, float fTargetAcc, float fTargetTime, int nMoveMode, float fBlendingRadius, int nBlendingType, int nSyncType); 
            int _movel(float fTargetPos[NUM_JOINT], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveMode, int nMoveReference, float fBlendingRadius, int nBlendingType, int nSyncType);             
            int _movec(float fTargetPos[2][NUM_TASK], float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveMode, int nMoveReference, float fBlendingRadius, int nBlendingType, int nSyncType);
            int _movejx(float fTargetPos[NUM_TASK], float fTargetVel, float fTargetAcc, float fTargetTime, int nMoveMode, int nMoveReference, float fBlendingRadius, int nBlendingType, int nSolSpace, int nSyncType);
            int _move_periodic(float fAmplitude[NUM_TASK], float fPeriodic[NUM_TASK], float fAccelTime, int nRepeat, int nMoveReference, int nSyncType);
            int _move_spiral(int nTaskAxis, float fRevolution, float fMaxRadius, float fMaxLength, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveReference, int nSyncType);
            int _movesj(float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT], int nPosCount, float fTargetVel, float fTargetAcc, float fTargetTime, int nMoveMode, int nSyncType);
            int _movesx(float fTargetPos[MAX_SPLINE_POINT][NUM_TASK], int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveMode, int nMoveReference, int nVelOpt, int nSyncType);
            int _moveb(MOVE_POSB* fTargetPos, int nPosCount, float fTargetVel[2], float fTargetAcc[2], float fTargetTime, int nMoveMode, int nMoveReference, int nSyncType);
            
            //void thread_subscriber();
            //void msgRobotState_cb(const dsr_msgs::RobotState::ConstPtr& msg);
            ///boost::thread m_thread_sub;

            std::string m_strSrvNamePrefix;
            std::string m_strTopicNamePrefix; 
    };
}
#endif // end