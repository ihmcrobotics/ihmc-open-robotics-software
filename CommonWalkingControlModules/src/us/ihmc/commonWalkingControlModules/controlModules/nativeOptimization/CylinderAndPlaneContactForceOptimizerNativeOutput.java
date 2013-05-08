package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

public class CylinderAndPlaneContactForceOptimizerNativeOutput
{
   private final DenseMatrix64F rho;
   private final DenseMatrix64F phi;
   private double optVal;
   private int numberOfIterations;

   public CylinderAndPlaneContactForceOptimizerNativeOutput(int rhoSize, int phiSize)
   {
      rho = new DenseMatrix64F(rhoSize, 1);
      phi = new DenseMatrix64F(phiSize, 1);
   }

   public void setRho(double[] rho)
   {
      for (int i = 0; i < this.rho.getNumRows(); i++)
         this.rho.set(i, 0, rho[i]);
   }
   
   public void setPhi(double[] phi)
   {
      for (int i=0;i<this.phi.getNumRows();i++)
         this.phi.set(i,0,phi[i]);
   }

   public void setOptVal(double optVal)
   {
      this.optVal = optVal;
   }

   public void setNumberOfIterations(int numberOfIterations)
   {
      this.numberOfIterations = numberOfIterations;
   }

   public DenseMatrix64F getRho()
   {
      return rho;
   }
   
   public DenseMatrix64F getPhi()
   {
      return phi;
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
