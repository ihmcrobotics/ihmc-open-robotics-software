package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;

public class HQPNativeOutput
{
   private final DenseMatrix64F vd;
   private final DenseMatrix64F rho;
   private final DenseMatrix64F phi;

   public HQPNativeOutput(int nDoF, int rhoSize, int phiSize)
   {
      vd = new DenseMatrix64F(nDoF, 1);
      rho = new DenseMatrix64F(rhoSize, 1);
      phi = new DenseMatrix64F(phiSize, 1);
   }

   public void setRho(double[] rho)
   {
      for (int i = 0; i < this.rho.getNumRows(); i++)
      {
         this.rho.set(i, 0, rho[i]);
      }
   }

   public void setPhi(double[] phi)
   {
      for (int i = 0; i < this.phi.getNumRows(); i++)
      {
         this.phi.set(i, 0, phi[i]);
      }
   }

   public void setJointAccelerations(double[] vd)
   {
      for (int i = 0; i < this.vd.getNumRows(); i++)
      {
         this.vd.set(i, 0, vd[i]);
      }
   }

   public DenseMatrix64F getJointAccelerations()
   {
      return vd;
   }

   public DenseMatrix64F getRho()
   {
      return rho;
   }

   public DenseMatrix64F getPhi()
   {
      return phi;
   }
}
