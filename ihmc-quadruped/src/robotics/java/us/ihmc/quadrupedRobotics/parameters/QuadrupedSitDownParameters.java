package us.ihmc.quadrupedRobotics.parameters;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;

public interface QuadrupedSitDownParameters
{
   WholeBodySetpointParameters getSitDownParameters();

   double getTimeToMoveForSittingDown();
}
