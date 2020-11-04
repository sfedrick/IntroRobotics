clear
clc



% start ROS 
con = rosStart(true); % start ROS with gripper enabled

% add command

q=[pi/4,0,0,0,0,0];
%q = [0,0,0, 0,0,0];

con = add_command(con,q);
disp("Now the current state value is : ...");
disp(con.cur_state);
disp("Finish the command!");

% shut down ROS
rosEnd; 