publicacion_pub = rospublisher("publicacion", "std_msgs/String");
loop_rate = rosrate(1);

count = 0;
disp("PRACTICA P-S")

while true
    msg = rosmessage("std_msgs/String");
    msg.Data = "Cuenta: " + count;
    disp(msg.Data + " seg");
    send(publicacion_pub, msg);
    waitfor(loop_rate);
    count = count + 1;
end

