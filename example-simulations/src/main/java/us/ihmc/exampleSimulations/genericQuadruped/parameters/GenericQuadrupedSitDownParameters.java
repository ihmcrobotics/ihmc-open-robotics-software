package us.ihmc.exampleSimulations.genericQuadruped.parameters;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedSitDownParameters;

public class GenericQuadrupedSitDownParameters implements QuadrupedSitDownParameters
{
   private final GenericQuadrupedSitDownSetpoints sitDownSetpoints = new GenericQuadrupedSitDownSetpoints();

   @Override
   public WholeBodySetpointParameters getSitDownParameters()
   {
      return sitDownSetpoints;
   }

   @Override
   public double getTimeToMoveForSittingDown()
   {
      return 1.5;
   }
}
