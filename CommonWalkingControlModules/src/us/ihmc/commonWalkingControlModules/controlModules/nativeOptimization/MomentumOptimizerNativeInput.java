package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class MomentumOptimizerNativeInput
{
   private final double[] A;
   private final double[] b;
   private final double[] C;
   private final double[] Js;
   private final double[] ps;
   private final double[] Ws;
   private final double[] Lambda;
   private final double[] Q;
   private final double[] c;
   private final double[] rhoMin;
   private final double[] N;
   private final double[] z;
   private double wRho;

   // conversion helpers
   private final DenseMatrix64F AMatrix;
   private final DenseMatrix64F bMatrix;
   private final DenseMatrix64F CMatrix;
   private final DenseMatrix64F JsMatrix;
   private final DenseMatrix64F psMatrix;
   private final DenseMatrix64F WsMatrix;
   private final DenseMatrix64F LambdaMatrix;
   private final DenseMatrix64F QMatrix;
   private final DenseMatrix64F cMatrix;
   private final DenseMatrix64F rhoMinMatrix;
   private final DenseMatrix64F NMatrix;
   private final DenseMatrix64F zMatrix;

   public MomentumOptimizerNativeInput()
   {
      int rhoSize = MomentumOptimizerNative.rhoSize;
      int wrenchLength = MomentumOptimizerNative.wrenchLength;
      int nDoF = MomentumOptimizerNative.nDoF;
      int nNull = MomentumOptimizerNative.nNull;

      AMatrix = new DenseMatrix64F(wrenchLength, nDoF);
      bMatrix = new DenseMatrix64F(wrenchLength, 1);
      CMatrix = new DenseMatrix64F(wrenchLength, wrenchLength);
      JsMatrix = new DenseMatrix64F(nDoF, nDoF);
      psMatrix = new DenseMatrix64F(nDoF, 1);
      WsMatrix = new DenseMatrix64F(nDoF, nDoF);
      LambdaMatrix = new DenseMatrix64F(nDoF, nDoF);
      QMatrix = new DenseMatrix64F(wrenchLength, rhoSize);
      cMatrix = new DenseMatrix64F(wrenchLength, 1);
      rhoMinMatrix = new DenseMatrix64F(rhoSize, 1);
      NMatrix = new DenseMatrix64F(nDoF, nNull);
      zMatrix = new DenseMatrix64F(nNull, 1);

      A = new double[AMatrix.getNumElements()];
      b = new double[bMatrix.getNumElements()];
      C = new double[CMatrix.getNumRows()];    // diagonal
      Js = new double[JsMatrix.getNumElements()];
      ps = new double[psMatrix.getNumElements()];
      Ws = new double[WsMatrix.getNumRows()];    // diagonal
      Lambda = new double[LambdaMatrix.getNumRows()];    // diagonal
      Q = new double[QMatrix.getNumElements()];
      c = new double[cMatrix.getNumElements()];
      rhoMin = new double[rhoMinMatrix.getNumElements()];
      N = new double[NMatrix.getNumElements()];
      z = new double[zMatrix.getNumElements()];
   }

   public double[] getA()
   {
      return A;
   }

   public double[] getB()
   {
      return b;
   }

   public double[] getC()
   {
      return C;
   }

   public double[] getJs()
   {
      return Js;
   }

   public double[] getPs()
   {
      return ps;
   }

   public double[] getWs()
   {
      return Ws;
   }

   public double[] getLambda()
   {
      return Lambda;
   }

   public double[] getQ()
   {
      return Q;
   }

   public double[] getc()
   {
      return c;
   }

   public double[] getRhoMin()
   {
      return rhoMin;
   }

   public double[] getN()
   {
      return N;
   }

   public double[] getZ()
   {
      return z;
   }

   public double getwRho()
   {
      return wRho;
   }

   public void setCentroidalMomentumMatrix(DenseMatrix64F A)
   {
      CommonOps.insert(A, this.AMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.AMatrix, this.A);
   }

   public void setMomentumDotEquationRightHandSide(DenseMatrix64F b)
   {
      CommonOps.insert(b, this.bMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.bMatrix, this.b);
   }

   public void setMomentumDotWeight(DenseMatrix64F C)
   {
      // diagonal
      CommonOps.insert(C, this.CMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.CMatrix, this.C);
   }

   public void setSecondaryConstraintJacobian(DenseMatrix64F Js)
   {
      CommonOps.insert(Js, this.JsMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.JsMatrix, this.Js);
   }

   public void setSecondaryConstraintRightHandSide(DenseMatrix64F ps)
   {
      CommonOps.insert(ps, this.psMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.psMatrix, this.ps);
   }

   public void setSecondaryConstraintWeight(DenseMatrix64F Ws)
   {
      // diagonal
      CommonOps.insert(Ws, this.WsMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.WsMatrix, this.Ws);
   }

   public void setJointAccelerationRegularization(DenseMatrix64F Lambda)
   {
      // diagonal
      CommonOps.insert(Lambda, this.LambdaMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.LambdaMatrix, this.Lambda);
   }

   public void setContactPointWrenchMatrix(DenseMatrix64F Q)
   {
      CommonOps.insert(Q, this.QMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.QMatrix, this.Q);
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

   public void setNullspaceMatrixTranspose(DenseMatrix64F N)
   {
      CommonOps.insert(N, this.NMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.NMatrix, this.N);
   }

   public void setNullspaceMultipliers(DenseMatrix64F z)
   {
      CommonOps.insert(z, this.zMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.zMatrix, this.z);
   }

   public void setGroundReactionForceRegularization(double wRho)
   {
      this.wRho = wRho;
   }
}
