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
joint2 = robotics.Joint('joint2', 'prismatic');
joint2.HomePosition = 0;
setFixedTransform(joint2, eye(4));
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
theta_range = linspace(0, 2*pi, 50); % Revolute joint angle (0 to 2*pi)
z_range = linspace(0, 1, 20); % Prismatic joint position (0 to 1)
r_range = linspace(0, 1, 20); % Radial prismatic joint position (0 to 1)

% Initialize an empty matrix to store reachable points
reachable_points = [];

% Loop through all possible joint configurations
for theta = theta_range
    for z = z_range
        for r = r_range
            % Compute forward kinematics for cylindrical configuration
            x = r * cos(theta);
            y = r * sin(theta);
            position = [x, y, z];

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
title('Workspace for Cylindrical Configuration');
axis([-1.5 1.5 -1.5 1.5 0 2]); % Set axis limits for X, Y, and Z

