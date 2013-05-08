package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.MatrixTools;

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
   private final DenseMatrix64F QrhoMatrix;
   private final DenseMatrix64F QphiMatrix;
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
      QrhoMatrix = new DenseMatrix64F(wrenchLength, rhoSize);
      QphiMatrix = new DenseMatrix64F(wrenchLength, phiSize);
      cMatrix = new DenseMatrix64F(wrenchLength, 1);
      rhoMinMatrix = new DenseMatrix64F(rhoSize, 1);
      phiMinMatrix = new DenseMatrix64F(phiSize, 1);
      phiMaxMatrix = new DenseMatrix64F(phiSize, 1);

      C = new double[CMatrix.getNumRows()]; 
      Qrho = new double[QrhoMatrix.getNumElements()];
      Qphi = new double[QphiMatrix.getNumElements()];
      c = new double[cMatrix.getNumElements()];
      rhoMin = new double[rhoMinMatrix.getNumElements()];
      phiMin = new double[phiMinMatrix.getNumElements()];
      phiMax = new double[phiMaxMatrix.getNumElements()];
   }

   public double[] getC()
   {
      return C;
   }

   public double[] getQrho()
   {
      return Qrho;
   }
   
   public double[] getQphi()
   {
      return Qphi;
   }

   public double[] getc()
   {
      return c;
   }

   public double[] getRhoMin()
   {
      return rhoMin;
   }
   
   public double[] getPhiMin()
   {
      return phiMin;
   }
   
   public double[] getPhiMax()
   {
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

   public void setMomentumDotWeight(DenseMatrix64F C)
   {
      // diagonal
      CommonOps.insert(C, this.CMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.CMatrix, this.C);
   }

   public void setContactPointWrenchMatrix(DenseMatrix64F Qrho)
   {
      CommonOps.insert(Qrho, this.QrhoMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.QrhoMatrix, this.Qrho);
   }
   
   public void setContactPointWrenchMatrixForBoundedCylinderVariables(DenseMatrix64F Qphi)
   {
      CommonOps.insert(Qphi, this.QphiMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.QphiMatrix, this.Qphi);
   }

   public void setWrenchEquationRightHandSide(DenseMatrix64F c)
   {
      CommonOps.insert(c, this.cMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.cMatrix, this.c);
   }

   public void setRhoMin(DenseMatrix64F rhoMin)
   {
      CommonOps.insert(rhoMin, this.rhoMinMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.rhoMinMatrix, this.rhoMin);
   }
   
   public void setPhiMin(DenseMatrix64F phiMin)
   {
      CommonOps.insert(phiMin, this.phiMinMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.phiMinMatrix, this.phiMin);
   }
   
   public void setPhiMax(DenseMatrix64F phiMax)
   {
      CommonOps.insert(phiMax, this.phiMaxMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.phiMaxMatrix, this.phiMax);
   }

   public void setGroundReactionForceRegularization(double wRho)
   {
      this.wRho = wRho;
   }
   
   public void setCylinderBoundedVectorRegularization(double wPhi)
   {
      this.wPhi=wPhi;
   }
}
