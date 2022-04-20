% clientes action
[gripper_client, gripper_goal] = rosactionclient('/gripper_controller/gripper_cmd');
[robot_client, robot_goal] = rosactionclient('/arm_controller/follow_joint_trajectory');
% servicio para gripper
gripper_on_client = rossvcclient("/ur5/gripper/on");
gripper_off_client = rossvcclient("/ur5/gripper/off");

% Nombres de articulaciones
joint_names = ["elbow_joint"; "shoulder_lift_joint"; "shoulder_pan_joint";...
    "wrist_1_joint"; "wrist_2_joint"; "wrist_3_joint"];
% Posiciones de robot (en radianes)
home =     [1.94; -2.36; -1.57; -1.15; -1.63; 0.0];
p1_lower = [1.5; -0.98; -0.74; -2.06; -1.63; 0.75];
p1_upper = [1.45; -1.05; -0.74; -1.97; -1.63; 0.88];
p2_lower = [1.5; -0.98; -2.51; -2.08; -1.57; -1.01];
p2_upper = [1.69; -1.24; -2.31; -2.06; -1.63; -0.75];
% Posicion de gripper
gripper_pos = 0.4;

% Preparar mensajes
% Preparación de mensaje para controlar gripper
gripper_goal.Command.Position = gripper_pos;
close = copy(gripper_goal); %Mensaje de cierre de gripper

open = copy(gripper_goal);
open.Command.Position = 0.0; %Mensaje de apertura de gripper

% Preparación de mensajes con posiciones, 2 segundos por posircion
joint_send = rosmessage('trajectory_msgs/JointTrajectoryPoint');
joint_send.TimeFromStart = [rosduration(2)];

robot_goal.Trajectory.JointNames = joint_names;
robot_goal.Trajectory.Points = joint_send;

home_goal = copy(robot_goal);
joint_send.Positions = home;
home_goal.Trajectory.Points = copy(joint_send);

p1_lower_goal = copy(robot_goal);
joint_send.Positions = p1_lower;
p1_lower_goal.Trajectory.Points = copy(joint_send);

p1_upper_goal = copy(robot_goal);
joint_send.Positions = p1_upper;
p1_upper_goal.Trajectory.Points = copy(joint_send);

p2_lower_goal = copy(robot_goal);
joint_send.Positions = p2_lower;
p2_lower_goal.Trajectory.Points = copy(joint_send);

p2_upper_goal = copy(robot_goal);
joint_send.Positions = p2_upper;
p2_upper_goal.Trajectory.Points = copy(joint_send);

% Abre gripper
call(gripper_off_client);
sendGoalAndWait(gripper_client, open);
% Posicion inicial
sendGoalAndWait(robot_client, home_goal);
% Aproximación 1
sendGoalAndWait(robot_client, p1_upper_goal);
% Toma 1
sendGoalAndWait(robot_client, p1_lower_goal);
% Cerrar gripperr
pause(0.5)
sendGoalAndWait(gripper_client, close);
call(gripper_on_client);
pause(0.5)
% Aproximación 1
sendGoalAndWait(robot_client, p1_upper_goal);
% Aproximación 2
sendGoalAndWait(robot_client, p2_upper_goal);
% Toma 2
sendGoalAndWait(robot_client, p2_lower_goal);
% Abrir gripper
pause(0.5)
sendGoalAndWait(gripper_client, open);
call(gripper_off_client);
pause(0.5)
% Aproximación 2
sendGoalAndWait(robot_client, p2_upper_goal);
% Posicion inicial
sendGoalAndWait(robot_client, home_goal);