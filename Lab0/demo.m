clear
clc

addpath('../Core') % references ROS interface and arm controller files you'll need for every lab

% start ROS 
con = rosStart(true); % start ROS with gripper enabled

% add command
q=[-pi/2,0,pi/4,0,pi/2,0];
%q=[pi/4,0,0,0,0,0];
%q = [0,0,0, 0,0,0];

con = add_command(con,q);
disp("Now the current state value is : ...");
disp(con.cur_state);
disp("Finish the command!");

% shut down ROS
rosEnd; 