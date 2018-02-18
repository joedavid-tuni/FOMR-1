function pos=getrobotpose(tftree)
% take map to robot transform
mapodom = getTransform(tftree, 'map', 'base_footprint');
% convert quaternian to euler angle
[yaw, pitch, roll] = quat2angle([mapodom.Transform.Rotation.W mapodom.Transform.Rotation.X mapodom.Transform.Rotation.Y mapodom.Transform.Rotation.Z]);
% set x y and yaw value to pose
pos=[mapodom.Transform.Translation.X mapodom.Transform.Translation.Y yaw];