package us.ihmc.commonWalkingControlModules.configurations;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public interface HighLevelControllerParameters
{
   WholeBodySetpointParameters getStandPrepParameters();

   JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelControllerName state);
   double getDesiredJointStiffness(String joint, HighLevelControllerName state);
   double getDesiredJointDamping(String joint, HighLevelControllerName state);

   HighLevelControllerName getDefaultInitialControllerState();
   HighLevelControllerName getFallbackControllerState();

   boolean automaticallyTransitionToWalkingWhenReady();

   double getTimeToMoveInStandPrep();
   double getMinimumTimeInStandReady();
   double getTimeInStandTransition();
   double getCalibrationDuration();
}