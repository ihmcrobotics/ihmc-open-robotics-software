package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedWaypointCallback
{
   void isDoneMoving(RobotQuadrant robotQuadrant, boolean doneMoving);
}
