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
setFixedTransform(joint2, axang2tform([1 0 0 pi/2]));
link2.Joint = joint2;
addBody(robot, link2, 'link1');

% Add the third link (link3) and joint (joint3)
link3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3', 'prismatic');
joint3.HomePosition = 0;
setFixedTransform(joint3, eye(4));
link3.Joint = joint3;
addBody(robot, link3, 'link2');

% Set joint limits
theta1_range = linspace(0, 2*pi, 50); % Revolute joint angle (0 to 2*pi)
theta2_range = linspace(-pi/2, pi/2, 50); % Revolute joint angle (-pi/2 to pi/2)
z_range = linspace(0.2, 1, 20); % Prismatic joint position (0 to 1)

% Initialize an empty matrix to store reachable points
reachable_points = [];

% Loop through all possible joint configurations
for theta1 = theta1_range
    for theta2 = theta2_range
        for z = z_range
            % Compute forward kinematics for RRP configuration
            x = cos(theta1) * (1 + z * cos(theta2));
            y = sin(theta1) * (1 + z * cos(theta2));
            position = [x, y, z * sin(theta2)];

            % Store the reachable points
            reachable_points = [reachable_points; position];
        end
    end
end

% Plot the reachable points in 3D
scatter3(reachable_points(:, 1), reachable_points(:, 2), reachable_points(:, 3), '.');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Workspace for Spherical Configuration');
axis([-2 2 -2 2 -1.5 1.5]); % Set axis limits for X, Y, and Z
