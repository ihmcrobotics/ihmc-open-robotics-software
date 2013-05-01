package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixFeatures;
import us.ihmc.utilities.screwTheory.Momentum;

/**
 * @author twan
 *         Date: 5/1/13
 */
public class MomentumOptimizationSettings
{
   private final DenseMatrix64F C = new DenseMatrix64F(Momentum.SIZE, Momentum.SIZE);
   private double wRho;
   private double lambda;

   public void setMomentumDotWeight(DenseMatrix64F C)
   {
      if (!MatrixFeatures.isDiagonalPositive(C))
         throw new RuntimeException("C not diagonal positive");

      this.C.set(C);
   }

   public void setGroundReactionForceRegularization(double wRho)
   {
      this.wRho = wRho;
   }

   public void setDampedLeastSquaresFactor(double lambda)
   {
      this.lambda = lambda;
   }

   public DenseMatrix64F getMomentumDotWeight()
   {
      return C;
   }

   public double getGroundReactionForceRegularization()
   {
      return wRho;
   }

   public double getDampedLeastSquaresFactor()
   {
      return lambda;
   }
}
