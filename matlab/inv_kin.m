model = importrobot('ik_gripper.urdf');
ik = generalizedInverseKinematics('RigidBodyTree', model);

pose=...
    [1.00  0.00 0.00 0.00;...
     0.00  -1.0 0.00 0.00;...
     0.0  0.00 -1.00 0.00;...
     0.00  0.00 0.00 1.00];

eePosition = [0.56 -0.38 0.025];
eePose = trvec2tform(eePosition);
finalpose = eePose*pose;

poseConst = constraintPoseTarget('gripper', ...
    'ReferenceBody', 'world',...
    'TargetTransform', finalpose);

initial = homeConfiguration(model);

initial(1).JointPosition = -0.88;
initial(2).JointPosition = -1.26;
initial(3).JointPosition = 1.57;
initial(4).JointPosition = -1.57;
initial(5).JointPosition = -1.51;
initial(6).JointPosition = 0.13;

ik.ConstraintInputs = {'pose'};
[configSol,solInfo] = ik(initial,poseConst);

%figure
show(model, configSol, 'Collisions', 'off', 'Visuals', 'off')
