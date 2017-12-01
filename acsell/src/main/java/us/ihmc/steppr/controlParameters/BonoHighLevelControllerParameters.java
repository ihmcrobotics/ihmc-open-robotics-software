package us.ihmc.steppr.controlParameters;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class BonoHighLevelControllerParameters implements HighLevelControllerParameters
{
   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return null;
   }

   @Override
   public JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelControllerName state)
   {
      return JointDesiredControlMode.EFFORT;
   }

   @Override
   public double getDesiredJointStiffness(String joint, HighLevelControllerName state)
   {
      return 0;
   }

   @Override
   public double getDesiredJointDamping(String joint, HighLevelControllerName state)
   {
      return 0;
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return false;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return 0;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 0;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 0;
   }

   @Override
   public double getCalibrationDuration()
   {
      return 0;
   }
}
