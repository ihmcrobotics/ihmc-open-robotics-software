package us.ihmc.darpaRoboticsChallenge.controllers;

import javax.vecmath.Point3d;

public interface OneClickStepPathPlanner
{
   public void planStraightLinePathToPoint(Point3d destinationInGlobalCoordinates);

}
