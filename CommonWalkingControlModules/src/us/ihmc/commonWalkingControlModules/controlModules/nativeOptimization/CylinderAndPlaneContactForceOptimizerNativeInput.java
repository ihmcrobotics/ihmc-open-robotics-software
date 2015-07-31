package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.linearAlgebra.MatrixTools;

public class CylinderAndPlaneContactForceOptimizerNativeInput
{
   private final double[] C;
   private final double[] Qrho;
   private final double[] Qphi;
   private final double[] c;
   private final double[] rhoMin;
   private final double[] phiMin;
   private final double[] phiMax;
   private double wPhi;
   private double wRho;

   // conversion helpers
   private final DenseMatrix64F CMatrix;
   private final DenseMatrix64F qRhoMatrix;
   private final DenseMatrix64F qPhiMatrix;
   private final DenseMatrix64F cMatrix;
   private final DenseMatrix64F rhoMinMatrix;
   private final DenseMatrix64F phiMinMatrix;
   private final DenseMatrix64F phiMaxMatrix;

   public CylinderAndPlaneContactForceOptimizerNativeInput()
   {
      int rhoSize = CylinderAndPlaneContactForceOptimizerNative.rhoSize;
      int phiSize = CylinderAndPlaneContactForceOptimizerNative.phiSize;
      int wrenchLength = CylinderAndPlaneContactForceOptimizerNative.wrenchLength;

      CMatrix = new DenseMatrix64F(wrenchLength, wrenchLength);
      qRhoMatrix = new DenseMatrix64F(wrenchLength, rhoSize);
      qPhiMatrix = new DenseMatrix64F(wrenchLength, phiSize);
      cMatrix = new DenseMatrix64F(wrenchLength, 1);
      rhoMinMatrix = new DenseMatrix64F(rhoSize, 1);
      phiMinMatrix = new DenseMatrix64F(phiSize, 1);
      phiMaxMatrix = new DenseMatrix64F(phiSize, 1);

      C = new double[CMatrix.getNumRows()];
      Qrho = new double[qRhoMatrix.getNumElements()];
      Qphi = new double[qPhiMatrix.getNumElements()];
      c = new double[cMatrix.getNumElements()];
      rhoMin = new double[rhoMinMatrix.getNumElements()];
      phiMin = new double[phiMinMatrix.getNumElements()];
      phiMax = new double[phiMaxMatrix.getNumElements()];
   }

   public double[] getC()
   {
      MatrixTools.extractDiagonal(this.CMatrix, this.C);

      return C;
   }

   public double[] getQrho()
   {
      MatrixTools.denseMatrixToArrayColumnMajor(this.qRhoMatrix, this.Qrho);

      return Qrho;
   }

   public double[] getQphi()
   {
      MatrixTools.denseMatrixToArrayColumnMajor(this.qPhiMatrix, this.Qphi);

      return Qphi;
   }

   public double[] getc()
   {
      MatrixTools.denseMatrixToArrayColumnMajor(this.cMatrix, this.c);

      return c;
   }

   public double[] getRhoMin()
   {
      MatrixTools.denseMatrixToArrayColumnMajor(this.rhoMinMatrix, this.rhoMin);

      return rhoMin;
   }

   public double[] getPhiMin()
   {
      MatrixTools.denseMatrixToArrayColumnMajor(this.phiMinMatrix, this.phiMin);

      return phiMin;
   }

   public double[] getPhiMax()
   {
      MatrixTools.denseMatrixToArrayColumnMajor(this.phiMaxMatrix, this.phiMax);

      return phiMax;
   }

   public double getwRho()
   {
      return wRho;
   }

   public double getwPhi()
   {
      return wPhi;
   }

   public void resetToZeros()
   {
      CMatrix.zero();
      qRhoMatrix.zero();
      qPhiMatrix.zero();
      cMatrix.zero();
      rhoMinMatrix.zero();
      phiMinMatrix.zero();
      phiMaxMatrix.zero();
      wPhi = 0;
      wRho = 0;
   }

   public void setMomentumDotWeight(DenseMatrix64F C)
   {
      // diagonal
      CommonOps.insert(C, this.CMatrix, 0, 0);
   }

   public void setWrenchEquationRightHandSide(DenseMatrix64F c)
   {
      CommonOps.insert(c, this.cMatrix, 0, 0);
   }

   public void setRhoMin(DenseMatrix64F rhoMin)
   {
      CommonOps.insert(rhoMin, this.rhoMinMatrix, 0, 0);
   }

   public void setPhiMin(DenseMatrix64F phiMin)
   {
      CommonOps.insert(phiMin, this.phiMinMatrix, 0, 0);
   }

   public void setPhiMax(DenseMatrix64F phiMax)
   {
      CommonOps.insert(phiMax, this.phiMaxMatrix, 0, 0);
   }

   public void setGroundReactionForceRegularization(double wRho)
   {
      this.wRho = wRho;
   }

   public void setCylinderBoundedVectorRegularization(double wPhi)
   {
      this.wPhi = wPhi;
   }

   public void setRhoMin(int rhoLocation, int i, double rhoMin2)
   {
      rhoMinMatrix.set(rhoLocation, i, rhoMin2);
   }

   public void setQRho(DenseMatrix64F qRho)
   {
      CommonOps.insert(qRho, this.qRhoMatrix, 0, 0);
   }

   public void setQPhi(DenseMatrix64F qPhi)
   {
      CommonOps.insert(qPhi, this.qPhiMatrix, 0, 0);
   }
   
   public String toString()
   {
      StringBuilder ret = new StringBuilder(1000);
      ret.append("CylinderAndPlaneContactForceOptimizerNativeInput with \n");
      appendMatrix(ret,"CMatrix",CMatrix);
      appendMatrix(ret,"QrhoMatrix", qRhoMatrix);
      appendMatrix(ret,"QphiMatrix", qPhiMatrix);
      appendMatrix(ret,"cMatrix",cMatrix);
      appendMatrix(ret,"rhoMinMatrix",rhoMinMatrix);
      appendMatrix(ret,"phiMinMatrix",phiMinMatrix);
      appendMatrix(ret,"phiMaxMatrix",phiMaxMatrix);
      ret.append("\twPhi "+wPhi+"\n");
      ret.append("\twRho "+wRho+"\n");
      
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
