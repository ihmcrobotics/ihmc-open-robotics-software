package us.ihmc.wanderer.parameters;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

public class WandererHighLevelControllerParameters implements HighLevelControllerParameters
{
   @Override
   public StandPrepParameters getStandPrepParameters()
   {
      return null;
   }

   @Override
   public JointDesiredControlMode getJointDesiredControlMode(String joint, HighLevelController state)
   {
      return JointDesiredControlMode.EFFORT;
   }

   @Override
   public double getDesiredJointStiffness(String joint)
   {
      return 0;
   }

   @Override
   public double getDesiredJointDamping(String joint)
   {
      return 0;
   }

   @Override
   public HighLevelController getDefaultInitialControllerState()
   {
      return HighLevelController.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public HighLevelController getFallbackControllerState()
   {
      return HighLevelController.DO_NOTHING_BEHAVIOR;
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
