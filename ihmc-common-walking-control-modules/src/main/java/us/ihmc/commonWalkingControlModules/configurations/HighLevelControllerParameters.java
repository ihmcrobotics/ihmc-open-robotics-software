package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public interface HighLevelControllerParameters
{
   StandPrepParameters getStandPrepParameters();

   JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelController state);
   double getDesiredJointStiffness(String joint);
   double getDesiredJointDamping(String joint);

   HighLevelController getDefaultInitialControllerState();
   HighLevelController getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
   double getTimeInStandTransition();
}