package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolver;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.MatrixMissingTools;

import static org.junit.jupiter.api.Assertions.*;

public class GarbageFreeDoglegSolverRosenbrock2DTest
{
   public static final double EPSILON = 1.0e-12;

   private GarbageFreeDoglegSolver solver;

   private final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {-2.0, 2.0});

   @BeforeEach
   public void setUpSolver()
   {
      double trustRegionRadius = 1.0;
      int maximumIterations = 50;

      solver = new GarbageFreeDoglegSolver(new Rosenbrock2DResidualAndJacobian(), maximumIterations);

      solver.initialize(x0, trustRegionRadius);
      solver.updateProblemObjects();
      solver.updateProblemSteps();
   }

   @Test
   public void testResidual()
   {
      DMatrixRMaj residual = solver.getResidual();

      DMatrixRMaj expected = new DMatrixRMaj(new double[] {-20.0, 3.0});

      boolean isEqual = MatrixFeatures_DDRM.isEquals(residual, expected, EPSILON);
      assertTrue(isEqual);
   }

   @Test
   public void testObjective()
   {
      double objective = solver.getObjective();

      double expected = 0.5 * (100.0 * MathTools.square(x0.get(1) - MathTools.square(x0.get(0))) + MathTools.square(1.0 - x0.get(0)));

      assertEquals(objective, expected, EPSILON);
   }

   @Test
   public void testJacobian()
   {
      DMatrixRMaj jacobian = solver.getJacobian();

      DMatrixRMaj expected = new DMatrixRMaj(new double[][] {{40.0, 10.0}, {-1.0, 0.0}});

      boolean isEqual = MatrixFeatures_DDRM.isEquals(jacobian, expected, EPSILON);
      assertTrue(isEqual);
   }

   @Test
   public void testGradient()
   {
      DMatrixRMaj gradient = solver.getGradient();

      DMatrixRMaj expected = new DMatrixRMaj(new double[] {-803.0, -200.0});

      boolean isEqual = MatrixFeatures_DDRM.isEquals(gradient, expected, EPSILON);
      assertTrue(isEqual);
   }

   @Test
   public void testAlpha()
   {
      double alpha = solver.getAlpha();

      double expected = 5.879101191917243e-4;

      assertEquals(alpha, expected, EPSILON);
   }

   @Test
   public void testSteepestDescentStep()
   {
      DMatrixRMaj steepestDescentStep = new DMatrixRMaj(2, 1);
      steepestDescentStep.set(solver.getSteepestDescentStep());

      DMatrixRMaj gradient = solver.getGradient(); // NOTE: assumes testGradient() is passing
      DMatrixRMaj expected = new DMatrixRMaj(2, 1);
      expected.set(gradient);
      MatrixMissingTools.negate(expected);

      boolean isEqual = MatrixFeatures_DDRM.isEquals(steepestDescentStep, expected, EPSILON);
      assertTrue(isEqual);
   }

   @Test
   public void testGaussNewtonStep()
   {
      DMatrixRMaj gaussNewtonStep = solver.getGaussNewtonStep();

      DMatrixRMaj jacobian = solver.getJacobian(); // NOTE: assumes testJacobian() is passing
      DMatrixRMaj residual = solver.getResidual(); // NOTE: assumes testResidual() is passing
      MatrixMissingTools.negate(residual);
      DMatrixRMaj expected = new DMatrixRMaj(2, 1);
      // We use an SVD based system solve here, as it's the most robust and performance isn't an issue. Perhaps we should change this to the solver we use
      // inside the NLLS solver, however
      LinearSolver<DMatrixRMaj, DMatrixRMaj> linearSystemSolver = LinearSolverFactory_DDRM.pseudoInverse(true);
      linearSystemSolver.setA(jacobian);
      linearSystemSolver.solve(residual, expected);

      boolean isEqual = MatrixFeatures_DDRM.isEquals(gaussNewtonStep, expected, EPSILON);
      assertTrue(isEqual);
   }

   @Test
   public void testSolve()
   {
      double trustRegionRadius = 1.0;
      solver.initialize(x0, trustRegionRadius);
      solver.setVerbose(true);
      solver.solve();

      assertTrue(solver.isConverged());

      DMatrixRMaj expected = new DMatrixRMaj(new double[] {1.0, 1.0});
      boolean isEqual = MatrixFeatures_DDRM.isEquals(solver.getX(), expected, EPSILON);
      assertTrue(isEqual);
   }

   private static class Rosenbrock2DResidualAndJacobian implements GarbageFreeResidualAndJacobian
   {
      int parameterSize = 2;
      int residualSize = 2;

      @Override
      public void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack)
      {
         double x1 = x.get(0);
         double x2 = x.get(1);
         residualToPack.unsafe_set(0, 0, 10.0 * (x2 - MathTools.square(x1)));
         residualToPack.unsafe_set(1, 0, 1.0 - x1);
      }

      @Override
      public void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack)
      {
         double x1 = x.get(0);
         double x2 = x.get(1);
         jacobianToPack.unsafe_set(0, 0, -20.0 * x1);
         jacobianToPack.unsafe_set(0, 1, 10.0);
         jacobianToPack.unsafe_set(1, 0, -1.0);
         jacobianToPack.unsafe_set(1, 1, 0.0);
      }

      @Override
      public int getParameterSize()
      {
         return parameterSize;
      }

      @Override
      public int getResidualSize()
      {
         return residualSize;
      }
   }
}
