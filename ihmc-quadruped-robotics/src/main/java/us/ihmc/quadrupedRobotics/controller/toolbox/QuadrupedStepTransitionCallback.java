package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedStepTransitionCallback
{
   void onLiftOff(RobotQuadrant robotQuadrant);

   void onTouchDown(RobotQuadrant robotQuadrant);
}
