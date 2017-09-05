package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.PositionControlParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public interface HighLevelControllerParameters
{
   StandPrepParameters getStandPrepParameters();

   PositionControlParameters getPositionControlParameters();

   LowLevelJointControlMode getLowLevelJointControlMode(String joint, NewHighLevelControllerStates state);

   NewHighLevelControllerStates getDefaultInitialControllerState();
   NewHighLevelControllerStates getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
}
