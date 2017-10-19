package us.ihmc.valkyrie;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.StandPrepParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelController;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;

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
   public JointDesiredControlMode getJointDesiredControlMode(String jointName, HighLevelController state)
   {
      throw new RuntimeException("None of the Valkyrie joint control modes have been set up yet.");
   }

   @Override
   public double getDesiredJointStiffness(String jointName)
   {
      throw new RuntimeException("None of the Valkyrie joint stiffnesses have been set up yet.");
   }

   @Override
   public double getDesiredJointDamping(String jointName)
   {
      throw new RuntimeException("None of the Valkyrie joint dampings have been set up yet.");
   }

   @Override
   public HighLevelController getDefaultInitialControllerState()
   {
      return HighLevelController.WALKING;
   }

   @Override
   public HighLevelController getFallbackControllerState()
   {
      return HighLevelController.DO_NOTHING_BEHAVIOR;
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
