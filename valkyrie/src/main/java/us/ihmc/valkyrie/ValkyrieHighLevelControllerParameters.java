package us.ihmc.valkyrie;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.PositionControlParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;

public class ValkyrieHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final ValkyrieStandPrepParameters standPrepParameters;
   private final ValkyriePositionControlParameters positionControlParameters;

   private final boolean runningOnRealRobot;

   public ValkyrieHighLevelControllerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      standPrepParameters = new ValkyrieStandPrepParameters();
      positionControlParameters = new ValkyriePositionControlParameters();
   }

   @Override
   public StandPrepParameters getStandPrepParameters()
   {
      return standPrepParameters;
   }

   @Override
   public LowLevelJointControlMode getLowLevelJointControlMode(String jointName, HighLevelControllerState state)
   {
      throw new RuntimeException("None of the Valkyrie joint control modes have been set up yet.");
   }

   @Override
   public HighLevelControllerState getDefaultInitialControllerState()
   {
      return HighLevelControllerState.WALKING;
   }

   @Override
   public HighLevelControllerState getFallbackControllerState()
   {
      return HighLevelControllerState.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return 5.0;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 3.0;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 0;
   }
}
