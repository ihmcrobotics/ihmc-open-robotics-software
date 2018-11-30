package us.ihmc.quadrupedRobotics.controller.toolbox;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedStepTransitionCallback
{
   void onLiftOff(QuadrupedTimedStep desiredStep);

   void onTouchDown(RobotQuadrant robotQuadrant);
}
