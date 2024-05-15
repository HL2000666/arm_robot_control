#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>

class MyRobot : public gazebo_ros_control::DefaultRobotHWSim
{
public:
  MyRobot() 
  { 
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_joint_1("joint_1", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(state_handle_joint_1);

    hardware_interface::JointStateHandle state_handle_joint_2("joint_2", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(state_handle_joint_2);

    hardware_interface::JointStateHandle state_handle_joint_3("joint_3", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(state_handle_joint_3);

    hardware_interface::JointStateHandle state_handle_joint_4("joint_4", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(state_handle_joint_2);

    hardware_interface::JointStateHandle state_handle_gripper_joint_1("gripper_joint_1", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(state_handle_gripper_joint_1);

    hardware_interface::JointStateHandle state_handle_gripper_joint_2("gripper_joint_2", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(state_handle_gripper_joint_2);

    registerInterface(&jnt_state_interface);

    // connect and register the joint position interface
    hardware_interface::JointHandle pos_handle_joint_1(jnt_state_interface.getHandle("joint_1"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_joint_1);

    hardware_interface::JointHandle pos_handle_joint_2(jnt_state_interface.getHandle("joint_2"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_joint_2);

    hardware_interface::JointHandle pos_handle_joint_3(jnt_state_interface.getHandle("joint_3"), &cmd[2]);
    jnt_pos_interface.registerHandle(pos_handle_joint_3);

    hardware_interface::JointHandle pos_handle_joint_4(jnt_state_interface.getHandle("joint_4"), &cmd[3]);
    jnt_pos_interface.registerHandle(pos_handle_joint_4);

    hardware_interface::JointHandle pos_handle_gripper_joint_1(jnt_state_interface.getHandle("gripper_joint_1"), &cmd[4]);
    jnt_pos_interface.registerHandle(pos_handle_joint_1);

    hardware_interface::JointHandle pos_handle_gripper_joint_2(jnt_state_interface.getHandle("gripper_joint_2"), &cmd[5]);
    jnt_pos_interface.registerHandle(pos_handle_joint_2);

    registerInterface(&jnt_pos_interface);
  }

  void readSim(ros::Time time, ros::Duration period) override
  {
    // Read joint states from Gazebo
    gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
  }

  void writeSim(ros::Time time, ros::Duration period) override
  {
    // Compute control signals and write joint commands to Gazebo
    computeControlSignals(time, period);
    gazebo_ros_control::DefaultRobotHWSim::writeSim(time, period);
  }

  void setReferencePosition(double time)
  {
    double amplitude = 1.0;  
    double frequency = 0.5;  
    double phase_shift = 0.0;  
    double offset = 0.0;  

    for (size_t i = 0; i < 6; ++i) // Assuming there are 2 joints
    {
        // Compute the reference position using the sine function
        ref_pos_[i] = amplitude * sin(2 * M_PI * frequency * time + phase_shift) + offset;
    }
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[6];
  double pos[6];
  double vel[6];
  double eff[6];
  std::vector<double> ref_pos_;

  void computeControlSignals(const ros::Time& time, const ros::Duration& period)
  {
    // Simple proportional control
    for (size_t i = 0; i < 6; ++i)
    {
      double error = ref_pos_[i] - pos[i];
      cmd[i] = error * 0.1; // Proportional gain
    }
  }
};
