package us.ihmc.darpaRoboticsChallenge.controllers;

import javax.vecmath.Point3d;

public interface SimpleFootstepPlacer
{
   public void planSingleStep(Point3d destinationInGlobalCoordinates);
}
