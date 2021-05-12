function [u_c, r_c] = vbap_slsv(usv_odom, rabbit_position)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle
    
% Gains
% Virtual leader
kvl = 2;
kdx = 0.0;
kdy = 0.0;
kr = 0.05;

% Rabbit - Virtual leader
xl = rabbit_position.Point.X;
yl = rabbit_position.Point.Y;

% USV State
x = usv_odom.Pose.Pose.Position.X;
y = usv_odom.Pose.Pose.Position.Y;
u = usv_odom.Twist.Twist.Linear.X;
q = [usv_odom.Pose.Pose.Orientation.W, usv_odom.Pose.Pose.Orientation.X, ...
    usv_odom.Pose.Pose.Orientation.Y, usv_odom.Pose.Pose.Orientation.Z];
e = quat2eul(q);
psi = e(1);

% Rotation matrix
R = [cos(psi), -sin(psi); sin(psi), cos(psi)];
% Inertial velocities
dotxI = R*[u; 0];

% Distance to leader
dist = sqrt( (xl - x)^2 + (yl - y)^2);

% Control effort as accelerations in inertial
ddotx = kvl * abs(xl-x)*(xl-x)/dist - kdx*dotxI(1);
ddoty = kvl * abs(yl-y)*(yl-y)/dist - kdy*dotxI(2);

% Control effort in body-frame
accelBody = R'*[ddotx; ddoty];


dt = 0.1;
u_c = u + accelBody(1)*0.1;
r_c = kr * accelBody(2);
fprintf("yl=%.2f, y=%.2f, yl-y=%.2f\n",yl, y, yl-y);
fprintf("dist=%.1f, ddotx=%.2f, ddoty=%.2f, accel_u=%.2f, accel_v=%.2f, u_c=%.2f, r_c=%.2f\n", ...
    dist, ddotx, ddoty, accelBody(1), accelBody(2), u_c, r_c);
return
