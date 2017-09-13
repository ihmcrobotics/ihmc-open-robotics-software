package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public interface HighLevelControllerParameters
{
   StandPrepParameters getStandPrepParameters();

   LowLevelJointControlMode getLowLevelJointControlMode(String joint, HighLevelControllerState state);

   HighLevelControllerState getDefaultInitialControllerState();
   HighLevelControllerState getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
   double getTimeInStandTransition();
}