package us.ihmc.commonWalkingControlModules.controlModules.nativeOptimization;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.utilities.math.MatrixTools;

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

   public MomentumOptimizerNativeInput()
   {
      int rhoSize = MomentumOptimizerNative.rhoSize;
      int wrenchLength = MomentumOptimizerNative.wrenchLength;
      int nDoF = MomentumOptimizerNative.nDoF;
      int nNull = MomentumOptimizerNative.nNull;

      A = new double[wrenchLength * nDoF];
      b = new double[wrenchLength];
      C = new double[wrenchLength];    // diagonal
      Js = new double[nDoF * nDoF];
      ps = new double[nDoF];
      Ws = new double[nDoF];    // diagonal
      Lambda = new double[nDoF];    // diagonal
      Q = new double[wrenchLength * rhoSize];
      c = new double[wrenchLength];
      rhoMin = new double[rhoSize];
      N = new double[nDoF * nNull];
      z = new double[nNull];
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
      MatrixTools.denseMatrixToArrayColumnMajor(A, this.A);
   }

   public void setMomentumDotEquationRightHandSide(DenseMatrix64F b)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(b, this.b);
   }

   public void setMomentumDotWeight(DenseMatrix64F c)
   {
      MatrixTools.extractDiagonal(c, this.c);
   }

   public void setSecondaryConstraintJacobian(DenseMatrix64F Js)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(Js, this.Js);
   }

   public void setSecondaryConstraintRightHandSide(DenseMatrix64F ps)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(ps, this.ps);
   }

   public void setSecondaryConstraintWeight(DenseMatrix64F Ws)
   {
      MatrixTools.extractDiagonal(Ws, this.Ws);
   }

   public void setJointAccelerationRegularization(DenseMatrix64F Lambda)
   {
      MatrixTools.extractDiagonal(Lambda, this.Lambda);
   }

   public void setContactPointWrenchMatrix(DenseMatrix64F Q)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(Q, this.Q);
   }

   public void setWrenchEquationRightHandSide(DenseMatrix64F c)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(c, this.c);
   }

   public void setRhoMin(DenseMatrix64F rhoMin)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(rhoMin, this.rhoMin);
   }

   public void setNullspaceMatrix(DenseMatrix64F N)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(N, this.N);
   }

   public void setNullspaceMultipliers(DenseMatrix64F z)
   {
      MatrixTools.denseMatrixToArrayColumnMajor(z, this.z);
   }

   public void setGroundReactionForceRegularization(double wRho)
   {
      this.wRho = wRho;
   }
}
