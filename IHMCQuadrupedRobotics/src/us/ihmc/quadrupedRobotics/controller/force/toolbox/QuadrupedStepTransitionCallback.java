package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedStepTransitionCallback
{
   void onLiftOff(RobotQuadrant robotQuadrant);

   void onTouchDown(RobotQuadrant robotQuadrant);
}
