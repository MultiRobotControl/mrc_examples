clear

% Globals
% We will declare global variables that this function and the callbacks
% can all access
% When we receive an odometry messge, save it here.
global USV_ODOM;  
% When we receive a rabbit posiition, save it here
global RABBIT_POSITION;

% Try to start ROS - if it is already started, restart
try
    rosinit
catch
    rosshutdown
    rosinit
end

% Subscribers
usv_sub = rossubscriber('/cora/sensors/p3d',@usv_odom_callback, ...
    'DataFormat', 'struct');
% Add another subscriber here for the rabbit!
rabbit_sub = rossubscriber('/rabbit',@rabbit_callback,...
    'DataFormat', 'struct');

% Setup Publisher
cmd_pub = rospublisher('/cora/cmd_vel','geometry_msgs/Twist');
cmd_msg = rosmessage(cmd_pub);

% Infinite loop
while true
    % Call a function to implement the VBAP algorithm.
    if isempty(RABBIT_POSITION) 
        disp('WARNING - Rabbit position is empty')
        u_c = 0.0;
        r_c = 0.0;
    elseif isempty(USV_ODOM) 
        disp('WARNING - USV odometry is empty')
        u_c = 0.0;
        r_c = 0.0;
    else
        %RABBIT_POSITION.Point.X = 20;
        %RABBIT_POSITION.Point.Y = -372;c
        [u_c, r_c] = vbap_slsv(USV_ODOM, RABBIT_POSITION);
    end
    
    
    % Publish the results
    cmd_msg.Linear.X = u_c;
    cmd_msg.Angular.Z = r_c;
    send(cmd_pub, cmd_msg);
    
    pause(0.1);
end

    

