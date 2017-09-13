package us.ihmc.wanderer.parameters;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public class WandererHighLevelControllerParameters implements HighLevelControllerParameters
{
   @Override
   public StandPrepParameters getStandPrepParameters()
   {
      return null;
   }

   @Override
   public LowLevelJointControlMode getLowLevelJointControlMode(String joint, HighLevelControllerState state)
   {
      return LowLevelJointControlMode.FORCE_CONTROL;
   }

   @Override
   public HighLevelControllerState getDefaultInitialControllerState()
   {
      return HighLevelControllerState.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public HighLevelControllerState getFallbackControllerState()
   {
      return HighLevelControllerState.DO_NOTHING_BEHAVIOR;
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
