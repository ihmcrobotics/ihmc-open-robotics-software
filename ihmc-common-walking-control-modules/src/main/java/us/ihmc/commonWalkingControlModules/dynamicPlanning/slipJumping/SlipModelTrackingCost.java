package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.trajectoryOptimization.LQCostFunction;

public class SlipModelTrackingCost implements LQCostFunction
{
   @Override
   public double getCost(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector, DenseMatrix64F desiredStateVector)
   {
      return 0;
   }

   @Override
   public void getCostStateGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                    DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostControlGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F desiredControlVector,
                                      DenseMatrix64F desiredStateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostStateHessian(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostControlHessian(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostStateGradientOfControlGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }

   @Override
   public void getCostControlGradientOfStateGradient(DenseMatrix64F controlVector, DenseMatrix64F stateVector, DenseMatrix64F matrixToPack)
   {

   }
}
