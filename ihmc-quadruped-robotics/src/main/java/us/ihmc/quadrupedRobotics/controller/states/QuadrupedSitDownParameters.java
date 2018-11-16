package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;

public interface QuadrupedSitDownParameters
{
   WholeBodySetpointParameters getSitDownParameters();

   double getTimeToMoveForSittingDown();
}
