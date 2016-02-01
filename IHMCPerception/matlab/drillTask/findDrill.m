function [drillCloud tableCloud]= findDrill(cloud, planeTolerance, clickedPoint)
    nPoint = size(cloud,2);

    [B, P, inliers] = ransacfitplane(cloud(1:3,:), planeTolerance, 1);
    hold on
    
    tableCloud = cloud(:,inliers);
    drillCloud = cloud(:,setdiff(1:nPoint,inliers));

    %% find drill axis
    planeNormal = B(1:3) * sign(B'*[clickedPoint;1]);
    planeNormal = planeNormal ./ norm(planeNormal);
    tableCenter = mean(cloud(1:3,inliers),2);

    debug=0;
    if debug
        cla
        hold on
        plotCloud(tableCloud);
        hTable=plotCloud(tableCloud);
        set(hTable,'MarkerFaceColor','r');

        drawLine = @(x,y,c) plot3([x(1) y(1)], [x(2),y(2)], [x(3),y(3)],c,'LineWidth',3);
        drawLine(tableCenter, tableCenter+planeNormal*0.3, 'b');
        plot3(tableCenter(1),tableCenter(2),tableCenter(3),'c.','MarkerSize',100)
        plot3(clickedPoint(1),clickedPoint(2),clickedPoint(3),'g.','MarkerSize',100)
        hold off
    end
    