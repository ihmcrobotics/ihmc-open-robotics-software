package us.ihmc.quadrupedRobotics.controller.force.toolbox;

import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public interface QuadrupedTimedStepTransitionCallback
{
   void onLiftOff(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState);

   void onTouchDown(RobotQuadrant robotQuadrant, QuadrantDependentList<ContactState> contactState);
}
