function [v_c, r_c] = vbap_slsv(usv_odom, rabbit_position)
% Function prototype for implementing 
% Virtual Body, Artificial Potential - Single Leader, Single Vehicle

% Just setting surge velocity and yaw-rate commands to zero.
v_c = 0.0;
r_c = 0.0;
return
