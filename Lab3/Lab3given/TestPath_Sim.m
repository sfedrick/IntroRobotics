%% Setup
clear all
close all
addpath('maps')
addpath('../Core')

%% Simulation Parameters

start = [0,0,0,0,0,0];
goal = [0,0,1.1,0,0,0];

map = loadmap('Map1.txt');

% Find collision-free path using RRT to get list of waypoints
[path] = rrt(map, start, goal);
[row,col]=size(path);
%[path] = astar(map, start, goal);

%start ROS
lynx = ArmController();
pause(1) % wait for setup
collision = false;

% iterate over target waypoints
for target_index = 1:length(path(:,1))
    q = path(target_index, :);
    disp("Goal:")
    disp(q)
    lynx.set_pos(q)
    reached_target = false;
    
    % Define relevant variables here:
    if(mod(target_index,10)==0)
        disp("Percent of path completed")
        percentcomplete=target_index/row
    end
    while ~reached_target
        % Check if robot is collided then wait
        
        collision = collision | lynx.is_collided();
        pause(0.1)
        error=0.1;
        % Add Student code here to decide if controller should send next
        % target or continue to wait. Do NOT add additional pauses to control
        % loop. You will likely want to use lynx.get_state() to decide when to
        % move to the next target.
        [pos, vel] = lynx.get_state();
        %disp("position difference")
        posDiff=norm(pos(1:5)-q(1:5));
        if(posDiff<error)
            reached_target = true;
        end

        % End of student code
    end
    % End control loop

    disp("Current Configuration:");
    [pos, vel] = lynx.get_state();
    disp(pos);
end
if collision
    disp("Robot collided during move")
else
    disp("No collision detected")
end

lynx.stop() % Shut down ROS interface
