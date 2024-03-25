package us.ihmc.parameterEstimation.solver;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class GarbageFreeDoglegSolverOsborne2Test
{
   public static final double EPSILON = 1.0e-12;

   private GarbageFreeDoglegSolver solver;

   private final DMatrixRMaj x0 = new DMatrixRMaj(new double[] {1.3, 0.65, 0.65, 0.7, 0.6, 3.0, 5.0, 7.0, 2.0, 4.5, 5.5});

   @BeforeEach
   public void setUpSolver()
   {
      double trustRegionRadius = 1.0;
      int maximumIterations = 50;

      solver = new GarbageFreeDoglegSolver(new Osborne2ResidualAndJacobian(), maximumIterations);

      solver.initialize(x0, trustRegionRadius);
   }

   @Test
   public void testSolve()
   {
      solver.setVerbose(true);
      solver.solve();

      assertTrue(solver.isConverged());

      // Because this test function is quite ill-scaled, we omit a check on the solution value (it's too tough to check without using a really loose EPSILON),
      // and instead check only the objective value at the solution, which we know and can test tightly
      double expectedObjective = 0.0200688681467739;
      assertEquals(solver.getObjective(), expectedObjective, EPSILON);
   }
}
