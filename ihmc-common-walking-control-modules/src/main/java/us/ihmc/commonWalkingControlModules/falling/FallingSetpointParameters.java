package us.ihmc.commonWalkingControlModules.falling;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;

public interface FallingSetpointParameters extends WholeBodySetpointParameters
{
   @Override
   default double getSetpoint(String jointName)
   {
      return getSetpoint(jointName, FallingTrialConfiguration.DEFAULT);
   }

   double getSetpoint(String jointName, FallingTrialConfiguration trialConfiguration);
}
