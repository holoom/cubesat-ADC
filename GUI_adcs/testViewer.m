clc,clear all,close all
viewer = HelperOrientationViewer();
x = 0 ;
pp = poseplot("MeshFileName", "last.STL");
a = gca
a.YAxisLocation = 'right'
while 1
    x=x + pi/180;
    q = eul2quat([pi+x 0 0]) ;
%     qualtmulti
%     viewer(quaternion(q(1),q(2),q(3),q(4));
    pause(0.05);
    set(pp, "Orientation", quaternion(q(1),q(2),q(3),q(4)));
    drawnow
end