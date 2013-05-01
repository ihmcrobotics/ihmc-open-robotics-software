package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.utilities.math.MatrixTools;

/**
 * @author twan
 *         Date: 5/1/13
 */
public class MomentumOptimizerNativeOutput
{
   private final DenseMatrix64F vd;
   private final DenseMatrix64F rho;
   private double optVal;
   private int numberOfIterations;

   public MomentumOptimizerNativeOutput()
   {
      vd = new DenseMatrix64F(MomentumOptimizerNative.nDoF, 1);
      rho = new DenseMatrix64F(MomentumOptimizerNative.rhoSize, 1);
   }

   public void setRho(double[] rho)
   {
      MatrixTools.setMatrixColumnFromArray(this.rho, 0, rho);
   }

   public void setJointAccelerations(double[] vd)
   {
      MatrixTools.setMatrixColumnFromArray(this.vd, 0, vd);
   }

   public void setOptVal(double optVal)
   {
      this.optVal = optVal;
   }

   public void setNumberOfIterations(int numberOfIterations)
   {
      this.numberOfIterations = numberOfIterations;
   }

   public DenseMatrix64F getJointAccelerations()
   {
      return vd;
   }

   public DenseMatrix64F getRho()
   {
      return rho;
   }

   public double getOptVal()
   {
      return optVal;
   }

   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }
}
