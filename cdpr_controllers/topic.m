    

pub = rospublisher('/cable_command','sensor_msgs/JointState');
    msg = rosmessage(pub);

while(1)


    msg.Name={'cable0','cable1','cable2''cable3','cable4','cable5','cable6','cable7'};
    msg.Effort=[800,800,800,800,800,800,800,800];
    msg.Velocity=[0,0,0,0,0,0,0,0];
    msg.Position=[0,0,0,0,0,0,0,0];
    send(pub,msg);
end
