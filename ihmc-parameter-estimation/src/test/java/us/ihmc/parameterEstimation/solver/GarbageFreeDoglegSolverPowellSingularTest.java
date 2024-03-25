package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class GarbageFreeDoglegSolverPowellSingularTest
{
   public static final double EPSILON = 1.0e-12;

   private GarbageFreeDoglegSolver solver;

   private final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {3.0, -1.0, 0.0, 1.0});

   @BeforeEach
   public void setUpSolver()
   {
      double trustRegionRadius = 1.0;
      int maximumIterations = 50;

      solver = new GarbageFreeDoglegSolver(new PowellSingularResidualAndJacobian(), maximumIterations);

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
}
