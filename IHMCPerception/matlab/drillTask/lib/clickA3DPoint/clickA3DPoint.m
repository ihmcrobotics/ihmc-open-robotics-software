function h = clickA3DPoint(pointCloud)
%CLICKA3DPOINT
%   H = CLICKA3DPOINT(POINTCLOUD) shows a 3D point cloud and lets the user
%   select points by clicking on them. The selected point is highlighted 
%   and its index in the point cloud will is printed on the screen. 
%   POINTCLOUD should be a 3*N matrix, represending N 3D points. 
%   Handle to the figure is returned.
%
%   other functions required:
%       CALLBACKCLICK3DPOINT  mouse click callback function
%       ROWNORM returns norms of each row of a matrix
%       
%   To test this function ... 
%       pointCloud = rand(3,100)*100;
%       h = clickA3DPoint(pointCloud);
% 
%       now rotate or move the point cloud and try it again.
%       (on the figure View menu, turn the Camera Toolbar on, ...)
%
%   To turn off the callback ...
%       set(h, 'WindowButtonDownFcn',''); 
%
%   by Babak Taati
%   http://rcvlab.ece.queensu.ca/~taatib
%   Robotics and Computer Vision Laboratory (RCVLab)
%   Queen's University
%   May 4, 2005 
%   revised Oct 30, 2007
%   revised May 19, 2009

if nargin ~= 1
    error('Requires one input arguments.')
end

if size(pointCloud, 1)~=3
    error('Input point cloud must be a 3*N matrix.');
end

% show the point cloud
h = gcf;
plot3(pointCloud(1,:), pointCloud(2,:), pointCloud(3,:), 'c.'); 
cameratoolbar('Show'); % show the camera toolbar
hold on; % so we can highlight clicked points without clearing the figure

% set the callback, pass pointCloud to the callback function
set(h, 'WindowButtonDownFcn', {@callbackClickA3DPoint, pointCloud}); 
