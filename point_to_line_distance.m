function [distance, closestPoint, proj] = point_to_line_distance(point, lineStart, lineDir)
% Calculate shortest distance between point and line
% point: column vector representing the point
% lineStart: column vector representing the start point of the line
% lineDir: column vector representing the direction of the line

% Calculate vector from line start to point
v = point - lineStart;

% Calculate projection of v onto line direction
proj = dot(v, lineDir);

% Calculate closest point on line to point
closestPoint = lineStart + proj*lineDir;

% Calculate distance between closest point and point
distance = norm(point - closestPoint);

end