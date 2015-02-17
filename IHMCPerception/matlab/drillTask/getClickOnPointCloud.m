function [selectedPointIndex selectedPoint]=getClickOnPointCloud(h, cloud)
cameratoolbar('Show'); % show the camera toolbar
set(h,'Tag','');
set(h, 'WindowButtonDownFcn', {@callbackClickA3DPoint, cloud}); 
waitfor(h,'Tag');
set(h,'WindowButtonDownFcn',{});
selectedPointIndex=evalin('base','selectedPointIndex_');
selectedPoint=evalin('base','selectedPoint_');

