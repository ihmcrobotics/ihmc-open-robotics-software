package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.recursiveController;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class ICPTimingCostFunctionEstimator
{
   private final static double positionCost = 10.0;
   private final static double gradientCost = 2.0;

   private final DenseMatrix64F A = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F b = new DenseMatrix64F(3, 1);
   private final DenseMatrix64F quadraticCoefficients = new DenseMatrix64F(3, 1);

   private final DenseMatrix64F A_new = new DenseMatrix64F(3, 3);
   private final DenseMatrix64F b_new = new DenseMatrix64F(3, 1);

   private int numberOfPoints = 0;

   private final LinearSolver<DenseMatrix64F> solver = LinearSolverFactory.linear(0);

   public void addPoint(double costToGo, double costToGoGradient, double time)
   {
      double time4 = Math.pow(time, 4.0);
      double time3 = Math.pow(time, 3.0);
      double time2 = Math.pow(time, 2.0);

      double a00 = 2.0 * positionCost * time4 + 8.0 * gradientCost * time2;
      double a01 = 2.0 * positionCost * time3 + 4.0 * gradientCost * time;
      double a02 = 2.0 * positionCost * time2;

      double a10 = 2.0 * positionCost * time3 + 4.0 * gradientCost * time;
      double a11 = 2.0 * positionCost * time2 + 2.0 * gradientCost;
      double a12 = 2.0 * positionCost * time;

      double a20 = 2.0 * positionCost * time2;
      double a21 = 2.0 * positionCost * time;
      double a22 = 2.0 * positionCost;

      A_new.set(0, 0, a00);
      A_new.set(0, 1, a01);
      A_new.set(0, 2, a02);

      A_new.set(1, 0, a10);
      A_new.set(1, 1, a11);
      A_new.set(1, 2, a12);

      A_new.set(2, 0, a20);
      A_new.set(2, 1, a21);
      A_new.set(2, 2, a22);

      double b0 = 2.0 * positionCost * costToGo * time2 + 4.0 * gradientCost * costToGoGradient * time;
      double b1 = 2.0 * positionCost * costToGo * time + 2.0 * gradientCost * costToGoGradient;
      double b2 = 2.0 * positionCost * costToGo;

      b_new.set(0, 0, b0);
      b_new.set(1, 0, b1);
      b_new.set(2, 0, b2);

      CommonOps.addEquals(A, A_new);
      CommonOps.addEquals(b, b_new);

      numberOfPoints++;
   }

   public void addPoint(double costToGo, double time)
   {
      double time4 = Math.pow(time, 4.0);
      double time3 = Math.pow(time, 3.0);
      double time2 = Math.pow(time, 2.0);

      double a00 = 2.0 * positionCost * time4;
      double a01 = 2.0 * positionCost * time3;
      double a02 = 2.0 * positionCost * time2;

      double a10 = 2.0 * positionCost * time3;
      double a11 = 2.0 * positionCost * time2;
      double a12 = 2.0 * positionCost * time;

      double a20 = 2.0 * positionCost * time2;
      double a21 = 2.0 * positionCost * time;
      double a22 = 2.0 * positionCost;

      A_new.set(0, 0, a00);
      A_new.set(0, 1, a01);
      A_new.set(0, 2, a02);

      A_new.set(1, 0, a10);
      A_new.set(1, 1, a11);
      A_new.set(1, 2, a12);

      A_new.set(2, 0, a20);
      A_new.set(2, 1, a21);
      A_new.set(2, 2, a22);

      double b0 = 2.0 * positionCost * costToGo * time2;
      double b1 = 2.0 * positionCost * costToGo * time;
      double b2 = 2.0 * positionCost * costToGo;

      b_new.set(0, 0, b0);
      b_new.set(1, 0, b1);
      b_new.set(2, 0, b2);

      CommonOps.addEquals(A, A_new);
      CommonOps.addEquals(b, b_new);

      numberOfPoints++;
   }

   public double getEstimatedCostFunctionSolution()
   {
      if (numberOfPoints < 3)
         return 0.0;

      solver.setA(A);
      solver.solve(b, quadraticCoefficients);

      return -quadraticCoefficients.get(1) / (2.0 * quadraticCoefficients.get(0));
   }

   public void reset()
   {
      A.zero();
      b.zero();
      quadraticCoefficients.zero();

      numberOfPoints = 0;
   }
}
