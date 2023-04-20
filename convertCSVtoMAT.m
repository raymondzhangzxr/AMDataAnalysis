function convertCSVtoMAT(folder, filename)
% Load the data from the CSV file
isEyeTarget = false;
filePath = fullfile(folder, filename);
data = readtable(filePath);

% Extract the columns of interest
timestamp = data.Timestamp;
transX = data.TransX;
transY = data.TransY;
transZ = data.TransZ;
rotX = data.RotX;
rotY = data.RotY;
rotZ = data.RotZ;
rotW = data.RotW;

% Check if EyeTarget exists (Eye Target only exsist in HMD csv)
if ismember('EyeTarget', data.Properties.VariableNames)
    isEyeTarget = true;
    eyeTarget = data.EyeTarget;
else
    isEyeTarget = false;
end
% Convert the quaternion to rotation matrices
quat = [rotW, rotX, rotY, rotZ];
rotMats = quat2rotm(quat);

% Combine the rotation and translation data into transformation matrices
numSamples = length(timestamp);
transMats = repmat(eye(4), [1, 1, numSamples]);
for i = 1:numSamples
    transMats(:,:,i) = [rotMats(:,:,i), [transX(i); transY(i); transZ(i)]; 0 0 0 1];
end

% Save the time stamp and corresponding transformation matrices to a MAT file
[~, baseFileName, ~] = fileparts(filename);
matFileName = [baseFileName, '.mat'];
matFilePath = fullfile(matFileName);
if isEyeTarget
    save(matFilePath, 'timestamp', 'transMats', 'eyeTarget');
else
    save(matFilePath, 'timestamp', 'transMats');
end