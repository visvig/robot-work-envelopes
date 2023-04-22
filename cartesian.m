% Import robotics system toolbox
import robotics.*;

% Create a rigid body tree
robot = robotics.RigidBodyTree;

% Add the first link (link1) and joint (joint1)
link1 = robotics.RigidBody('link1');
joint1 = robotics.Joint('joint1', 'prismatic');
joint1.HomePosition = 0;
setFixedTransform(joint1, trvec2tform([0, 0, 0]));
link1.Joint = joint1;
addBody(robot, link1, 'base');

% Add the second link (link2) and joint (joint2)
link2 = robotics.RigidBody('link2');
joint2 = robotics.Joint('joint2', 'prismatic');
joint2.HomePosition = 0;
setFixedTransform(joint2, trvec2tform([0, 0, 0])); % Fixed transformation corrected
link2.Joint = joint2;
addBody(robot, link2, 'link1');

% Add the third link (link3) and joint (joint3)
link3 = robotics.RigidBody('link3');
joint3 = robotics.Joint('joint3', 'prismatic');
joint3.HomePosition = 0;
setFixedTransform(joint3, trvec2tform([0, 0, 0])); % Fixed transformation corrected
link3.Joint = joint3;
addBody(robot, link3, 'link2');

% Set joint limits
joint1_range = linspace(0, 1, 20);
joint2_range = linspace(0, 1, 20);
joint3_range = linspace(0, 1, 20);

% Initialize an empty matrix to store reachable points
reachable_points = [];

% Loop through all possible joint configurations
for q1 = joint1_range
    for q2 = joint2_range
        for q3 = joint3_range
            % Compute forward kinematics for Cartesian configuration
            position = [q1, q2, q3];

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
title('Workspace for Cartesian Configuration');
axis([-1.5 1.5 -1.5 1.5 0 2]); % Set axis limits for X, Y, and Z

