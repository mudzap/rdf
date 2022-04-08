#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTolerance.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

void add_waypoint(control_msgs::FollowJointTrajectoryGoal& goal,
		  double b, double a, double e, double w1, double w2, double w3,
		  double time) {
  goal.trajectory.points.resize(goal.trajectory.points.size() + 1);
  goal.trajectory.points.back().positions.resize(7); // Resize necesario para editar el vector
  goal.trajectory.points.back().positions[0] = b;
  goal.trajectory.points.back().positions[1] = a;
  goal.trajectory.points.back().positions[2] = e;
  goal.trajectory.points.back().positions[3] = w1;
  goal.trajectory.points.back().positions[4] = w2;
  goal.trajectory.points.back().positions[5] = w3;

  goal.trajectory.points.back().time_from_start = ros::Duration(time);

}

int main (int argc, char **argv) {
  ROS_INFO("Iniciado nodo");

  ros::init(argc, argv, "ur5_action");
  
  ROS_INFO("Definiendo cliente");
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/pos_joint_traj_controller/follow_joint_trajectory", true);

  ROS_INFO("Esperando a servidor");
  ac.waitForServer(ros::Duration(5.0));

  //Se define un objetivo
  // Este consiste de una trajectoria (JointTrajectory)
  // Un conjunto de tolerancias para cada articulación (JointTolerance) durante la ejecución de la trayectoria
  // Un conjunto de toleracias para cada articulación despues del tiempo máximo
  // Cada trayectoria consiste en si en un arreglo de puntos
  ROS_INFO("Definiendo objetivo");
  control_msgs::FollowJointTrajectoryGoal goal;
  
  goal.trajectory.joint_names.push_back("shoulder_pan_joint");
  goal.trajectory.joint_names.push_back("shoulder_lift_joint");
  goal.trajectory.joint_names.push_back("elbow_joint");
  goal.trajectory.joint_names.push_back("wrist_1_joint");
  goal.trajectory.joint_names.push_back("wrist_2_joint");
  goal.trajectory.joint_names.push_back("wrist_3_joint");
  
  ROS_INFO("Definiendo waypoint");
  add_waypoint(goal, -1.57, -0.7, -1.5, 0.0, 0.0, 0.0, 2.0);
  add_waypoint(goal, 1.57, -0.7, -1.5, 0.0, 0.0, 0.0, 4.0);
  add_waypoint(goal, -1.57, -0.7, -1.5, 0.0, 0.0, 0.0, 6.0);
  add_waypoint(goal, -1.57, -0.7, -1.5, 1.0, 1.5, 0.0, 8.0);
  add_waypoint(goal, -1.57, -0.7, -1.5, 1.0, 1.5, 0.0, 8.5);
  add_waypoint(goal, -1.73, -1.75, -1.84, 0.44, 1.73, 0.0, 10.0);
  add_waypoint(goal, -0.80, -1.75, -1.84, 0.44, 0.80, 0.0, 12.0);
  add_waypoint(goal, -0.80, -2.32, -2.04, 1.19, 0.80, 0.0, 14.0);
  add_waypoint(goal, -1.73, -2.32, -2.04, 1.19, 1.73, 0.0, 16.0);
  add_waypoint(goal, -1.73, -1.75, -1.84, 0.44, 1.73, 0.0, 18.0);
  add_waypoint(goal, -1.73, -1.75, -1.84, 0.44, 1.73, 0.0, 19.0);
  add_waypoint(goal, -1.57, -0.7, -1.5, 1.0, 1.5, 0.0, 21.0);

  ROS_INFO("Enviando objetivo");
  ac.sendGoal(goal);
  ROS_INFO("Esperando resultado por 25s");
  bool res = ac.waitForResult(ros::Duration(25.0)); 

  if(res) {
    ROS_INFO("Se recibio resultado");
    control_msgs::FollowJointTrajectoryResultConstPtr result;
    result = ac.getResult();
    ROS_INFO("Codigo de error: %x", result->error_code);
  } else {
    ROS_INFO("No se recibio resultado");
  }

  return 0;
}
