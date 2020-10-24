clear all
close all
addpath('maps')
addpath('../Core')

lynx = ArmController();
pause(1) % wait for setup
q=[0,1.4,.79,0,0,0];
lynx.set_pos(q)
 disp("Current Configuration:");
[pos, vel] = lynx.get_state()
disp(pos)
lynx.stop() % Shut down ROS interface
