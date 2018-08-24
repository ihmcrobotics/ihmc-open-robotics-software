package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;

public class GenericQuadrupedHighLevelControllerParameters implements HighLevelControllerParameters
{
   private final GenericQuadrupedStandPrepParameters standPrepParameters = new GenericQuadrupedStandPrepParameters();

   @Override
   public WholeBodySetpointParameters getStandPrepParameters()
   {
      return standPrepParameters;
   }

   @Override
   public HighLevelControllerName getDefaultInitialControllerState()
   {
      return HighLevelControllerName.WALKING;
   }

   @Override
   public HighLevelControllerName getFallbackControllerState()
   {
      return HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   }

   @Override
   public boolean automaticallyTransitionToWalkingWhenReady()
   {
      return true;
   }

   @Override
   public double getTimeToMoveInStandPrep()
   {
      return 4.0;
   }

   @Override
   public double getMinimumTimeInStandReady()
   {
      return 0.5;
   }

   @Override
   public double getTimeInStandTransition()
   {
      return 4.0;
   }

   @Override
   public double getCalibrationDuration()
   {
      throw new RuntimeException("Generic quadruped doesn't have a calibration state");
   }
}
