
%% Load and display robot
clear
clc

addpath(genpath(strcat(pwd,'\Dependencies')))
robot = createRigidBodyTree;
ax = show(robot);  % 'ax' will be the axes handle where the robot is displayed
ax.CameraPositionMode = 'auto';

%% Create a set of desired waypoints
waypointType = 'feeding'; % New waypoint type for feeding task
switch waypointType
    case 'feeding'
        % Define waypoints for the feeding task
        wayPoints = [0.3 0 0.2;  % Above the plate
                     0.3 0 0.1;  % Lowered onto the plate
                     0.3 0 0.2;  % Lifting from plate
                     0.15 0 0.25; % Intermediate point for smooth trajectory
                      0 0 0.2;    % In front of mouth
                     0 0 0.15;   % Feeding motion
                     0.3 0 0.2]; % Return to starting position
        % Define appropriate velocities for each waypoint
        wayPointVels = [0 0 0; 0 0.1 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0; 0 0 0];
end

%% Plot waypoints in the same axes as the robot
hold(ax, 'on');  % Hold the current axes to plot on top of the robot
plot3(ax, wayPoints(:,1), wayPoints(:,2), wayPoints(:,3), 'o-', ...
      'MarkerSize', 8, 'MarkerFaceColor', 'b', 'LineWidth', 2);
xlabel(ax, 'X');
ylabel(ax, 'Y');
zlabel(ax, 'Z');
hold(ax, 'off');  % Release the hold after plotting

%% Create a smooth trajectory from the waypoints
numTotalPoints = size(wayPoints,1)*10;
waypointTime = 4;
trajType = 'cubic'; % or 'trapezoidal'
switch trajType
    case 'trapezoidal'
        trajectory = trapveltraj(wayPoints',numTotalPoints,'EndTime',waypointTime);
    case 'cubic'
        wpTimes = (0:size(wayPoints,1)-1)*waypointTime;
        trajTimes = linspace(0,wpTimes(end),numTotalPoints);
        trajectory = cubicpolytraj(wayPoints',wpTimes,trajTimes, ...
                     'VelocityBoundaryCondition',wayPointVels');
end

% Plot trajectory spline and waypoints
hold on
plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

%% Perform Inverse Kinematics
% Use desired weights for solution (First three are orientation, last three are translation)
% Since it is a 4-DOF robot with only one revolute joint in Z we do not
% put a weight on Z rotation; otherwise it limits the solution space

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 1:size(trajectory,2)
    tform = trvec2tform(trajectory(:,idx)');
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

%% Visualize robot configurations
title('Robot waypoint tracking visualization')
axis([-0.1 0.4 -0.35 0.35 0 0.35]);
for idx = 1:size(trajectory,2)
    show(robot,configSoln(idx,:), 'PreservePlot', false,'Frames','off');
    pause(0.1)
end
hold off