package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;

import static org.junit.jupiter.api.Assertions.*;

public class DoglegSolverPowellSingularTest
{
   public static final double EPSILON = 1.0e-12;

   private DoglegSolver solver;

   private final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {3.0, -1.0, 0.0, 1.0});

   @BeforeEach
   public void setUpSolver()
   {
      double trustRegionRadius = 1.0;
      int maximumIterations = 50;

      solver = new DoglegSolver(new PowellSingularResidualAndJacobian(), maximumIterations);

      solver.initialize(x0, trustRegionRadius);
   }

   @Test
   public void testSolve()
   {
      solver.setVerbose(true);
      solver.solve();

      assertTrue(solver.isConverged());

      DMatrixRMaj expectedSolution = new DMatrixRMaj(new double[] {0.0, 0.0, 0.0, 0.0});
      // We relax the similarity EPSILON for this as it is a nearly singular problem
      boolean isEqual = MatrixFeatures_DDRM.isEquals(solver.getX(), expectedSolution, 1.0e-3);
      assertTrue(isEqual);

      double expectedObjective = 0.0;
      assertEquals(solver.getObjective(), expectedObjective, EPSILON);
   }

   /**
    * See More, Garbow, and Hillstrom (1981): "Testing Unconstrained Optimization Software".
    * <p>
    * This is the Powell singular function problem for nonlinear least squares solvers.
    * </p>
    */
   private static class PowellSingularResidualAndJacobian implements ResidualAndJacobian
   {
      int parameterSize = 4;
      int residualSize = 4;

      @Override
      public void calculateResidual(DMatrixRMaj x, DMatrixRMaj residualToPack)
      {
         residualToPack.unsafe_set(0, 0, x.get(0) + 10.0 * x.get(1));
         residualToPack.unsafe_set(1, 0, Math.sqrt(5.0) * (x.get(2) - x.get(3)));
         residualToPack.unsafe_set(2, 0, MathTools.square(x.get(1) - 2.0 * x.get(2)));
         residualToPack.unsafe_set(3, 0, Math.sqrt(10.0) * MathTools.square(x.get(0) - x.get(3)));
      }

      @Override
      public void calculateJacobian(DMatrixRMaj x, DMatrixRMaj jacobianToPack)
      {
         jacobianToPack.zero();
         // Only some Jacobian entries are non-zero
         // First row
         jacobianToPack.unsafe_set(0, 0, 1.0);
         jacobianToPack.unsafe_set(0, 1, 10.0);
         // Second row
         jacobianToPack.unsafe_set(1, 2, Math.sqrt(5.0));
         jacobianToPack.unsafe_set(1, 3, -Math.sqrt(5.0));
         // Third row
         jacobianToPack.unsafe_set(2, 1, 2.0 * (x.get(1) - 2.0 * x.get(2)));
         jacobianToPack.unsafe_set(2, 2, 8.0 * x.get(2) - 4.0 * x.get(1));
         // Fourth row
         jacobianToPack.unsafe_set(3, 0, Math.sqrt(10.0) * 2.0 * (x.get(0) - x.get(3)));
         jacobianToPack.unsafe_set(3, 3, Math.sqrt(10.0) * -2.0 * (x.get(0) - x.get(3)));
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
