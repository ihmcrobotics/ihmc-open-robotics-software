package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

public class StaticEquilibriumSolverTest
{
   private static final boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize", "true"));
   private static final double marginToTest = 0.02;

   @Disabled
   @Test
   public void testTriangleFlat()
   {
      runTest(StaticEquilibriumSolverInputExamples.createTriangleFlatGround());
   }

   @Disabled
   @Test
   public void testTriangleLowAngle()
   {
      runTest(StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly());
   }

   @Disabled
   @Test
   public void testTriangleHighAngle()
   {
      runTest(StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot());
   }

   @Disabled
   @Test
   public void testFlatSquare()
   {
      runTest(StaticEquilibriumSolverInputExamples.createFlatSquare());
   }

   @Disabled
   @Test
   public void testBipedFeet()
   {
      runTest(StaticEquilibriumSolverInputExamples.createBipedFeet());
   }

   @Disabled
   @Test
   public void testBipedFeetWithHandhold()
   {
      runTest(StaticEquilibriumSolverInputExamples.createBipedFeetWithHandhold());
   }

   @Disabled
   @Test
   public void testMatricesForFlatGround()
   {
      double comX = -0.5;
      double comY = -0.5;

      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createFlatSquare();
      StaticEquilibriumSolver solver = new StaticEquilibriumSolver();
      solver.initialize(input);

      DMatrixRMaj Aeq = solver.getAeq();
      DMatrixRMaj beq = solver.getBeq();

      // test corner
      DMatrixRMaj solution = new DMatrixRMaj(0);
      int numberOfVariables = input.getNumberOfContacts() * 4 + 2;
      solution.reshape(numberOfVariables, 1);

      double verticalForce = StaticEquilibriumSolver.mass * input.getGravityMagnitude();
      input.getSurfaceNormals().get(0).changeFrame(ReferenceFrame.getWorldFrame());
      double mu = input.getCoefficientOfFriction();
      double betaZ = 1.0 / Math.sqrt(MathTools.square(mu) + 1.0);
      double rhoAtCorner = verticalForce / (4.0 * betaZ);

      CommonOps_DDRM.fill(solution, 0.0);
      solution.set(0, rhoAtCorner);
      solution.set(1, rhoAtCorner);
      solution.set(2, rhoAtCorner);
      solution.set(3, rhoAtCorner);
      solution.set(numberOfVariables - 2, comX);
      solution.set(numberOfVariables - 1, comY);

      DMatrixRMaj b = new DMatrixRMaj(0);
      CommonOps_DDRM.mult(Aeq, solution, b);

      for (int i = 0; i < 6; i++)
      {
         double eps = 1e-8;
         Assertions.assertTrue(Math.abs(b.get(i) - beq.get(i)) < eps);
      }
   }

   private void runTest(StaticEquilibriumSolverInput input)
   {
      StaticEquilibriumSolver solver = new StaticEquilibriumSolver();
      solver.initialize(input);
      solver.solve();

      ConvexPolygon2D supportPolygon = solver.getSupportRegion();

      StaticEquilibriumForceOptimizer forceOptimizer = new StaticEquilibriumForceOptimizer();

      // test the vertices are valid
      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = supportPolygon.getVertex(i);
         System.out.println("testing vertex is valid:   " + vertex.getX() + "," + vertex.getY());
         boolean succeeded = forceOptimizer.solve(input, vertex);
         Assertions.assertTrue(succeeded);
//         System.out.println(succeeded ? "pass" : "fail");
      }

      System.out.println();
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      ConvexPolygon2D scaledPolygon = new ConvexPolygon2D();
      scaler.scaleConvexPolygon(supportPolygon, -marginToTest, scaledPolygon);

      // test outside the vertices by a margin aren't valid
      for (int i = 0; i < scaledPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = scaledPolygon.getVertex(i);
         System.out.println("testing vertex is invalid: " + vertex);
         boolean succeeded = forceOptimizer.solve(input, vertex);
         Assertions.assertFalse(succeeded);
//         System.out.println(!succeeded ? "pass" : "fail");
      }
   }

   private static void runTimingTest()
   {
      StaticEquilibriumSolver solver = new StaticEquilibriumSolver();

      StaticEquilibriumSolverInput input0 = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly();
      StaticEquilibriumSolverInput input1 = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyIn();
      StaticEquilibriumSolverInput input2 = StaticEquilibriumSolverInputExamples.createBipedFeet();
      StaticEquilibriumSolverInput input3 = StaticEquilibriumSolverInputExamples.createBipedFeetWithHandhold();

      // warm up
      for (int i = 0; i < 10; i++)
      {
         solver.initialize(input0);
         solver.solve();
      }

      // do timing test
      long start = System.currentTimeMillis();

      solver.initialize(input3);
      solver.solve();

      long stop = System.currentTimeMillis();

      long diff = stop - start;
      System.out.println("Solver time: " + diff + "milli-seconds") ;
   }

   public static void main(String[] args)
   {
      runTimingTest();
   }
}
