%% Constans
inputFolder = '4_19Free';
kwireFile = '3Kwire';
sceneFile = '3DemoScene';
eyeHMDFile = '3HMDAndEyeTarget';
toolTipOffset = [0.27572; -0.003;0];
sceneOffset = [0;0.006;0];
errorLowerLimit = 0.003;
errorUpperLmit = 0.01;
%% Read CSV files into MATs
convertCSVtoMAT(inputFolder, fullfile([kwireFile, '.csv']));
convertCSVtoMAT(inputFolder, fullfile([sceneFile, '.csv']));
convertCSVtoMAT(inputFolder, fullfile([eyeHMDFile, '.csv']));

%% Load MATs: 
load(fullfile([kwireFile, '.mat']));
kwireTime = timestamp;
kwirePoses = transMats;
load(fullfile([sceneFile, '.mat']));
sceneTime = timestamp;
scenePoses = transMats;
load(fullfile([eyeHMDFile, '.mat']));
hmdTime = timestamp;
hmdPoses = transMats;
ET = eyeTarget;

%% Draw the tip w.r.t time and the target line or target cylinder 
sceneTrans = scenePoses(1:3,4,1) + sceneOffset;
sceneRot = scenePoses(1:3, 1:3);
% Extract 3D translation vectors from transformation matrices
kWireT = squeeze(kwirePoses(1:3, 4, :));
% Extract rotation matrix from Kwire pose
kWireR = kwirePoses(1:3, 1:3, :);
% Rotate tool tip offset vector to world coordinate system
toolTipPosition = zeros(3, size(kwirePoses, 3));
for i = 1:size(kwirePoses, 3)
    toolTipPosition(:, i) = kWireR(:,:,i) * toolTipOffset;
end
toolTipPositionWorld = kWireT + toolTipPosition;
% Plot translations as red dots in 3D space
% !!!!key point!!!!
% To make a plot just like the view from Hololens2 HMD, we need to switch 
% Y and Z axis in MATLAB
figure
scatter3(toolTipPositionWorld (1,:), toolTipPositionWorld (3,:), toolTipPositionWorld (2,:), 'r.');
hold on

% % Draw the target line
% % Define line in local coordinate system

% Here are two plot possibilities, one is to plot a target line, the other
% is to plot a cylinder with 3mm diameter

% Code to define the target and plot the target line
targetOrigin = sceneTrans;
targetDirection = [0; 1; 1] / sqrt(2); % Unit vector in direction of line
lineLength = 0.15; % Desired length of line in m

% Transform line into world coordinates
lineDirectionWorld = sceneRot * targetDirection;
linePositionWorld = targetOrigin + lineDirectionWorld * lineLength;
% plot
% line([targetOrigin(1), linePositionWorld(1)], [targetOrigin(3), linePositionWorld(3)], [targetOrigin(2), linePositionWorld(2)], 'Color', 'g', 'LineWidth', 2);

% Code to plot a cylinder
% Define rotation matrix to align cylinder with desired direction
desiredDir = [0; 1; 1] / sqrt(2);% Dir of the target
cylinderRadius = 0.0015; % Radius of the cylinder 
originalDir = [0; 1; 0]; % Dir of the MATLAB original cylinder
rotAxis = cross(originalDir, desiredDir);
rotAxis = rotAxis / norm(rotAxis);
rotAngle = acos(dot(originalDir, desiredDir));
Rcylinder = vrrotvec2mat([rotAxis' rotAngle]);

% Generate cylinder
[cylinderX, cylinderY, cylinderZ] = cylinder(cylinderRadius);
cylinderZ = cylinderZ * cylinderHeight;

% Apply rotation matrix to cylinder coordinates
cylinderCoords = [cylinderX(:)'; cylinderZ(:)'; cylinderY(:)'];
cylinderCoords = Rcylinder * cylinderCoords;

% Transform cylinder into world coordinates
cylinderWorld = sceneRot * cylinderCoords;
cylinderWorld = cylinderWorld + targetOrigin;

% Plot cylinder
hold on;
cylinderColor = [0, 1, 0]; % Solid green outline
cylinderFaceAlpha = 0.5; % Semi-transparent green filling
surface(reshape(cylinderWorld(1,:),size(cylinderX)),reshape(cylinderWorld(3,:),size(cylinderX)),reshape(cylinderWorld(2,:),size(cylinderX)),...
    'FaceColor', cylinderColor, 'FaceAlpha', cylinderFaceAlpha, 'EdgeColor', cylinderColor);

hold on;
%% Error Calculation



% Initialize variables
valid_distances = [];

% Loop through each K-wire pose
for i = 1:size(toolTipPositionWorld, 2)
    % Calculate the shortest distance between the K-wire pose and the target line
    [distance, closestPoint, proj] = point_to_line_distance(toolTipPositionWorld(:, i), targetOrigin, lineDirectionWorld);
    
    % If the distance is within the valid range, and it's within the 
    % line's length add it to the accuracy
    if (distance > errorLowerLimit) && (distance < errorUpperLimit) && (proj < lineLength) 
        valid_distances = [valid_distances; distance];
        % Plot blue line from K-wire pose to closest point on target line
        plot3([toolTipPositionWorld(1, i), closestPoint(1)], [toolTipPositionWorld(3, i), closestPoint(3)], [toolTipPositionWorld(2, i), closestPoint(2)],'b-', 'LineWidth', 1);
        hold on;
    end
end

fprintf('Averaged Accumulated Error: %.4f mm\n', 1000 * sum(valid_distances)/ size(valid_distances, 1));
ax = gca;
ax.View = [-10.5, 12]; % Rotate view to desired orientation
ax.XDir = 'normal'; % Normal direction of x axis
ax.YDir = 'normal'; % Normal direction of y axis
ax.ZDir = 'normal'; % Normal direction of z axis

xlabel('X');
ylabel('Z');
zlabel('Y');
title('Kwire Tip and The Target');
legend('KWire Tip', 'Target Alignment Zone','Valid Distances', 'Location', 'northwest', 'FontSize', 6);