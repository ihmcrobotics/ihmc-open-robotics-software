package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

public class CylinderAndPlaneContactForceOptimizerNativeOutput
{
   private final DenseMatrix64F rho;
   private final DenseMatrix64F phi;
   private double optVal;
   private int numberOfIterations;

   public CylinderAndPlaneContactForceOptimizerNativeOutput()
   {
      rho = new DenseMatrix64F(CylinderAndPlaneContactForceOptimizerNative.rhoSize, 1);
      phi = new DenseMatrix64F(CylinderAndPlaneContactForceOptimizerNative.phiSize, 1);
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
   
   public String toString()
   {
      StringBuilder ret = new StringBuilder(1000);
      ret.append("CylinderAndPlaneContactForceOptimizerNativeOutput with \n");
      appendMatrix(ret,"rho",rho);
      appendMatrix(ret,"phi",phi);
      
      return ret.toString();
      
   }

   private void appendMatrix(StringBuilder ret, String string, DenseMatrix64F matrix)
   {
      ret.append("\t"+string+" = \n");
      for (int r=0;r<matrix.numRows;r++)
      {
         ret.append("\t\t[");
         for (int c=0;c<matrix.numCols;c++)
         {
            ret.append(String.format("%+.7e ", matrix.get(r, c)));
            
         }
         ret.append("]\n");
      }
   }

}
