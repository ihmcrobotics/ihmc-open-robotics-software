package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.utilities.math.linearAlgebra.MatrixTools;

public class CVXMomentumOptimizerWithGRFPenalizedSmootherNativeInput
{
   private final double[] A;
   private final double[] b;
   private final double[] C;
   private final double[] Js;
   private final double[] ps;
   private final double[] Ws;
   private final double[] Lambda;
   private final double[] Qrho;
   private final double[] c;
   private final double[] rhoMin;
   private final double[] WRho;
   private final double[] rhoPrevious;
   private final double[] WRhoSmoother;
   private final double[] rhoPreviousMean;
   private final double[] WRhoCoPPenalty;

   // conversion helpers
   private final DenseMatrix64F AMatrix;
   private final DenseMatrix64F bMatrix;
   private final DenseMatrix64F CMatrix;
   private final DenseMatrix64F JsMatrix;
   private final DenseMatrix64F psMatrix;
   private final DenseMatrix64F WsMatrix;
   private final DenseMatrix64F LambdaMatrix;
   private final DenseMatrix64F QrhoMatrix;
   private final DenseMatrix64F cMatrix;
   private final DenseMatrix64F rhoMinMatrix;
   private final DenseMatrix64F WRhoMatrix;
   private final DenseMatrix64F rhoPreviousMatrix;
   private final DenseMatrix64F WRhoSmootherMatrix;
   private final DenseMatrix64F rhoPreviousMeanMatrix;
   private final DenseMatrix64F WRhoCoPPenaltyMatrix;

   public CVXMomentumOptimizerWithGRFPenalizedSmootherNativeInput()
   {
      int rhoSize = CVXMomentumOptimizerWithGRFSmootherNative.rhoSize;
      int wrenchLength = CVXMomentumOptimizerWithGRFSmootherNative.wrenchLength;
      int nDoF = CVXMomentumOptimizerWithGRFSmootherNative.nDoF;

      AMatrix = new DenseMatrix64F(wrenchLength, nDoF);
      bMatrix = new DenseMatrix64F(wrenchLength, 1);
      CMatrix = new DenseMatrix64F(wrenchLength, wrenchLength);
      JsMatrix = new DenseMatrix64F(nDoF, nDoF);
      psMatrix = new DenseMatrix64F(nDoF, 1);
      WsMatrix = new DenseMatrix64F(nDoF, nDoF);
      LambdaMatrix = new DenseMatrix64F(nDoF, nDoF);
      QrhoMatrix = new DenseMatrix64F(wrenchLength, rhoSize);
      cMatrix = new DenseMatrix64F(wrenchLength, 1);
      rhoMinMatrix = new DenseMatrix64F(rhoSize, 1);
      WRhoMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      rhoPreviousMatrix = new DenseMatrix64F(rhoSize, 1);
      WRhoSmootherMatrix = new DenseMatrix64F(rhoSize, rhoSize);
      rhoPreviousMeanMatrix = new DenseMatrix64F(rhoSize, 1);
      WRhoCoPPenaltyMatrix = new DenseMatrix64F(rhoSize, rhoSize);

      A = new double[AMatrix.getNumElements()];
      b = new double[bMatrix.getNumElements()];
      C = new double[CMatrix.getNumRows()];    // diagonal
      Js = new double[JsMatrix.getNumElements()];
      ps = new double[psMatrix.getNumElements()];
      Ws = new double[WsMatrix.getNumRows()];    // diagonal
      Lambda = new double[LambdaMatrix.getNumRows()];    // diagonal
      Qrho = new double[QrhoMatrix.getNumElements()];
      c = new double[cMatrix.getNumElements()];
      rhoMin = new double[rhoMinMatrix.getNumElements()];
      WRho = new double[WRhoMatrix.getNumRows()];    // diagonal
      rhoPrevious = new double[rhoPreviousMatrix.getNumElements()];
      WRhoSmoother = new double[WRhoSmootherMatrix.getNumRows()];    // diagonal
      rhoPreviousMean = new double[rhoPreviousMeanMatrix.getNumElements()];
      WRhoCoPPenalty = new double[WRhoCoPPenaltyMatrix.getNumRows()];    // diagonal
   }

   public void reset()
   {
      AMatrix.zero();
      bMatrix.zero();
      CMatrix.zero();
      JsMatrix.zero();
      psMatrix.zero();
      WsMatrix.zero();
      LambdaMatrix.zero();
      QrhoMatrix.zero();
      cMatrix.zero();
      rhoMinMatrix.zero();
      WRhoMatrix.zero();
      rhoPreviousMatrix.zero();
      WRhoSmootherMatrix.zero();
      rhoPreviousMeanMatrix.zero();
      WRhoCoPPenaltyMatrix.zero();

      Arrays.fill(A, 0.0);
      Arrays.fill(b, 0.0);
      Arrays.fill(C, 0.0);
      Arrays.fill(Js, 0.0);
      Arrays.fill(ps, 0.0);
      Arrays.fill(Ws, 0.0);
      Arrays.fill(Lambda, 0.0);
      Arrays.fill(Qrho, 0.0);
      Arrays.fill(c, 0.0);
      Arrays.fill(rhoMin, 0.0);
      Arrays.fill(WRho, 0.0);
      Arrays.fill(rhoPrevious, 0.0);
      Arrays.fill(WRhoSmoother, 0.0);
      Arrays.fill(rhoPreviousMean, 0.0);
      Arrays.fill(WRhoCoPPenalty, 0.0);
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

   public double[] getQrho()
   {
      return Qrho;
   }

   public double[] getc()
   {
      return c;
   }

   public double[] getRhoMin()
   {
      return rhoMin;
   }

   public double[] getWRho()
   {
      return WRho;
   }

   public double[] getRhoPrevious()
   {
      return rhoPrevious;
   }

   public double[] getWRhoSmoother()
   {
      return WRhoSmoother;
   }

   public double[] getRhoPreviousMean()
   {
      return rhoPreviousMean;
   }

   public double[] getWRhoCoPPenalty()
   {
      return WRhoCoPPenalty;
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

   public void setContactPointWrenchMatrix(DenseMatrix64F Qrho)
   {
      CommonOps.insert(Qrho, this.QrhoMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.QrhoMatrix, this.Qrho);
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

   public void setGroundReactionForceRegularization(DenseMatrix64F WRho)
   {
      // diagonal
      CommonOps.insert(WRho, this.WRhoMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.WRhoMatrix, this.WRho);
   }

   public void setRhoPrevious(DenseMatrix64F rhoPrevious)
   {
      CommonOps.insert(rhoPrevious, this.rhoPreviousMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.rhoPreviousMatrix, this.rhoPrevious);
   }

   public void setRateOfChangeOfGroundReactionForceRegularization(DenseMatrix64F WRhoSmoother)
   {
      // diagonal
      CommonOps.insert(WRhoSmoother, this.WRhoSmootherMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.WRhoSmootherMatrix, this.WRhoSmoother);
   }

   public void setRhoPreviousAverage(DenseMatrix64F rhoPreviousMean)
   {
      CommonOps.insert(rhoPreviousMean, this.rhoPreviousMeanMatrix, 0, 0);
      MatrixTools.denseMatrixToArrayColumnMajor(this.rhoPreviousMeanMatrix, this.rhoPreviousMean);
   }

   public void setCenterOfPressurePenalizedRegularization(DenseMatrix64F WRhoCoPPenalty)
   {
      // diagonal
      CommonOps.insert(WRhoCoPPenalty, this.WRhoCoPPenaltyMatrix, 0, 0);
      MatrixTools.extractDiagonal(this.WRhoCoPPenaltyMatrix, this.WRhoCoPPenalty);
   }

}
