subscriber = rossubscriber("/joint_states", "sensor_msgs/JointState", @jointCallback);

function jointCallback(src, msg)
    len = size(msg.Position);
    for i = 1:len
        disp("Articulacion: " + msg.Name{i})
    end
    for i = 1:len
        disp("Posicion: " + msg.Position(i))
    end
end