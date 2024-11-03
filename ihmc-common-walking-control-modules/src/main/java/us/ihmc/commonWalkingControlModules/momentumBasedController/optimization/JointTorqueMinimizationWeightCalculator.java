package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrix;

import us.ihmc.yoVariables.registry.YoRegistry;

public interface JointTorqueMinimizationWeightCalculator
{
   void computeWeightMatrix(DMatrix tauMatrix, DMatrix weightMatrixToPack);

   default boolean isWeightZero()
   {
      return false;
   }

   default YoRegistry getRegistry()
   {
      return null;
   }
}
