package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public interface HighLevelControllerParameters
{
   StandPrepParameters getStandPrepParameters();

   LowLevelJointControlMode getLowLevelJointControlMode(String joint, HighLevelController state);
   double getLowLevelJointStiffness(String joint);
   double getLowLevelJointDamping(String joint);

   HighLevelController getDefaultInitialControllerState();
   HighLevelController getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
   double getTimeInStandTransition();
}