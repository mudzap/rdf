disp("Definiendo cliente")
ac = rosactionclient("/arm_controller/follow_joint_trajectory", "control_msgs/FollowJointTrajectory");

disp("Esperando a servidor")
waitForServer(ac, 5);

disp("Definiendo objetivo");
goal = rosmessage("control_msgs/FollowJointTrajectoryGoal");

goal.Trajectory.JointNames = ["shoulder_pan_joint";"shoulder_lift_joint"; ...
    "elbow_joint"; "wrist_1_joint"; "wrist_2_joint"; "wrist_3_joint"];
 
disp("Definiendo waypoint")
pos = [0.0; -0.7,; -1.5; 0.0; 0.0; 0.0];
time = rosduration(5.0);
point = rosmessage("trajectory_msgs/JointTrajectoryPoint");

point.Positions = pos;
point.TimeFromStart = time;
goal.Trajectory.Points = point;

disp("Enviando objetivo");

ac.ActivationFcn = @activeCb;
ac.FeedbackFcn = @feedbackCb;
ac.ResultFcn = @doneCb;
sendGoal(ac, goal);


function doneCb(src, res)
    disp("Se recibio resultado: ")
    disp("Codigo de error: " + res.Message.ErrorCode)
end

function activeCb(src)
end

function feedbackCb(src, fb)
    disp("Current pos:")
    disp(fb.Actual.Positions)
end