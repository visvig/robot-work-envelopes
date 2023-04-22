% Import robotics system toolbox
import robotics.*;

% Create a rigid body tree
robot = robotics.RigidBodyTree;

% Add the first link (link1) and joint (joint1)
link1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'revolute');
joint1.HomePosition = 0;
setFixedTransform(joint1, eye(4));
link1.Joint = joint1;
addBody(robot, link1, 'base');

% Add the second link (link2) and joint (joint2)
link2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2', 'revolute');
joint2.HomePosition = 0;
setFixedTransform(joint2, eye(4));
link2.Joint = joint2;
addBody(robot, link2, 'link1');

% Add the third link (link3) and joint (joint3)
link3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3', 'revolute');
joint3.HomePosition = 0;
setFixedTransform(joint3, eye(4));
link3.Joint = joint3;
addBody(robot, link3, 'link2');

% Set joint limits
theta1_range = linspace(-pi, pi, 20); % Revolute joint angle (-pi to pi)
theta2_range = linspace(-5*pi/6, 5*pi/6, 50); % Revolute joint angle (-pi to pi)
theta3_range = linspace(-5*pi/6, 5*pi/6, 20); % Revolute joint angle (-pi to pi)

% Set link lengths
L1 = 1; L2 = 1; L3 = 0.5;

% Initialize an empty matrix to store reachable points
reachable_points = [];

% Loop through all possible joint configurations
for theta1 = theta1_range
   for theta2 = theta2_range
       for theta3 = theta3_range
           % Compute forward kinematics for the articulated robot configuration
           x = L1 * cos(theta1) + L2 * cos(theta2) * cos(theta1) + L3 * cos(theta2 + theta3) * cos(theta1);
           y = L1 * sin(theta1) + L2 * cos(theta2) * sin(theta1) + L3 * cos(theta2 + theta3) * sin(theta1);
           z = L2 * sin(theta2) + L3 * sin(theta2 + theta3);
          
           % Store the reachable points
           position = [x, y, z];
           reachable_points = [reachable_points; position];
       end
   end
end

% Plot the reachable points in 3D
scatter3(reachable_points(:, 1), reachable_points(:, 2), reachable_points(:, 3), '.');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Workspace for Articulated Configuration');
axis([-3 3 -3 3 -3 3]); % Set axis limits for X, Y, and Z

