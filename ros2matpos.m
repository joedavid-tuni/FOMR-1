function pos=ros2matpos(rpos,tftree)

mapodom = getTransform(tftree, 'map', 'odom');
mapodom.Transform.Translation;
mapodom.Transform.Rotation;

[yaw, pitch, roll] = quat2angle([rpos.Pose.Pose.Orientation.W rpos.Pose.Pose.Orientation.X rpos.Pose.Pose.Orientation.Y rpos.Pose.Pose.Orientation.Z]);
pos=[rpos.Pose.Pose.Position.X rpos.Pose.Pose.Position.Y yaw];