function msg=mat2rospos(msg,pos)
% set x y and yaw value to mesaage which will be sent to robot
msg.Pose.Position.X=pos(1);
msg.Pose.Position.Y=pos(2);
msg.Pose.Position.Z=0;
% robot in 2d , angels refers as yaw angle
% convert to this angle to quaternian
q=angle2quat(pos(3),0,0);
% set quaternians
msg.Pose.Orientation.X=q(2);
msg.Pose.Orientation.Y=q(3);
msg.Pose.Orientation.Z=q(4);
msg.Pose.Orientation.W=q(1);
