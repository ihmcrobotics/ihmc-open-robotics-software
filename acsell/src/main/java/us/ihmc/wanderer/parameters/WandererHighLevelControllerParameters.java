package us.ihmc.wanderer.parameters;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public class WandererHighLevelControllerParameters implements HighLevelControllerParameters
{
   @Override
   public StandPrepParameters getStandPrepParameters()
   {
      return null;
   }

   @Override
   public LowLevelJointControlMode getLowLevelJointControlMode(String joint, HighLevelController state)
   {
      return LowLevelJointControlMode.FORCE_CONTROL;
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
}
