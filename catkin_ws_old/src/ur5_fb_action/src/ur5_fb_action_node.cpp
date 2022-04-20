#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTolerance.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <stdio.h>

void doneCb(const actionlib::SimpleClientGoalState& state,
  const control_msgs::FollowJointTrajectoryResultConstPtr& res) {
    ROS_INFO("Se recibio resultado");
    ROS_INFO("Codigo de error: %x", res->error_code);
}

void activeCb() {}

void feedbackCb(const control_msgs::FollowJointTrajectoryFeedbackConstPtr& fb) {
  printf("Current pos:");
  for(const auto& pos: fb->actual.positions) {
    printf(" %f", pos);
  }
  printf("\n");
}

int main (int argc, char **argv) {
  ROS_INFO("Iniciado nodo");

  ros::init(argc, argv, "ur5_action_fb");
  
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
  goal.trajectory.points.resize(1); // Resize necesario para editar el vector
  goal.trajectory.points[0].positions.resize(7); // Resize necesario para editar el vector
  goal.trajectory.points[0].positions[0] = 0.0;
  goal.trajectory.points[0].positions[1] = -0.7; // Levantar el brazo
  goal.trajectory.points[0].positions[2] = -1.5; // Flexionar este
  goal.trajectory.points[0].positions[3] = 0.0;
  goal.trajectory.points[0].positions[4] = 0.0;
  goal.trajectory.points[0].positions[5] = 0.0;

  goal.trajectory.points[0].time_from_start = ros::Duration(2.0);


  ROS_INFO("Enviando objetivo");
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ros::spin();

  return 0;
}
