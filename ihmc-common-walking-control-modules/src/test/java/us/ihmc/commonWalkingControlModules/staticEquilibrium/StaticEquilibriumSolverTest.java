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
   private static final boolean VISUALIZE = false; //  Boolean.parseBoolean(System.getProperty("visualize", "true"));
   private static final double marginToTest = 0.02;

   @Test
   public void testTriangleFlat()
   {
      runTest(StaticEquilibriumSolverInputExamples.createTriangleFlatGround());
   }

   @Test
   public void testTriangleLowAngle()
   {
      runTest(StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly());
   }

   @Test
   public void testTriangleHighAngle()
   {
      runTest(StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot());
   }

   @Test
   public void testFlatSquare()
   {
      runTest(StaticEquilibriumSolverInputExamples.createFlatSquare());
   }

   @Test
   public void testBipedFeet()
   {
      runTest(StaticEquilibriumSolverInputExamples.createBipedFeet());
   }

   @Test
   public void testBipedFeetWithHandhold()
   {
      runTest(StaticEquilibriumSolverInputExamples.createBipedFeetWithSingleHandhold());
   }

   @Test
   public void testMatricesForFlatGround()
   {
      double comX = -0.5;
      double comY = -0.5;

      StaticEquilibriumSolverInput input = StaticEquilibriumSolverInputExamples.createFlatSquare();
      StaticSupportRegionSolver solver = new StaticSupportRegionSolver();
      solver.initialize(input);

      DMatrixRMaj Aeq = solver.getAeq();
      DMatrixRMaj beq = solver.getBeq();

      // test corner
      DMatrixRMaj solution = new DMatrixRMaj(0);
      int numberOfVariables = input.getNumberOfContacts() * 4 + 2;
      solution.reshape(numberOfVariables, 1);

      double verticalForce = StaticSupportRegionSolver.mass * input.getGravityMagnitude();
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
      StaticSupportRegionSolver solver = new StaticSupportRegionSolver();
      solver.initialize(input);
      solver.solve();

      ConvexPolygon2D supportPolygon = solver.getSupportRegion();
      StaticEquilibriumForceOptimizer forceOptimizer = new StaticEquilibriumForceOptimizer();

      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      ConvexPolygon2D innerPolygon = new ConvexPolygon2D();
      ConvexPolygon2D outerPolygon = new ConvexPolygon2D();
      scaler.scaleConvexPolygon(supportPolygon, marginToTest, innerPolygon);
      scaler.scaleConvexPolygon(supportPolygon, -marginToTest, outerPolygon);

      for (int i = 0; i < innerPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly innerVertex = innerPolygon.getVertex(i);
         boolean innerSucceed = forceOptimizer.solve(input, innerVertex);
         Assertions.assertTrue(innerSucceed);

         Point2DReadOnly outerVertex = outerPolygon.getVertex(i);
         boolean outerSucceed = forceOptimizer.solve(input, outerVertex);

         Assertions.assertFalse(outerSucceed);
      }
   }

   private static void runTimingTest()
   {
      StaticSupportRegionSolver solver = new StaticSupportRegionSolver();

      StaticEquilibriumSolverInput input0 = StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly();
      StaticEquilibriumSolverInput input1 = StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyIn();
      StaticEquilibriumSolverInput input2 = StaticEquilibriumSolverInputExamples.createBipedFeet();
      StaticEquilibriumSolverInput input3 = StaticEquilibriumSolverInputExamples.createBipedFeetWithSingleHandhold();
      StaticEquilibriumSolverInput input4 = StaticEquilibriumSolverInputExamples.createBipedFeetWithTwoHandholds();

      StaticEquilibriumSolverInput[] inputs = new StaticEquilibriumSolverInput[]{input0, input1, input2, input3, input4};

      // warm up
      int warmups = 10;
      for (int i = 0; i < warmups; i++)
      {
         solver.initialize(input0);
         solver.solve();
      }

      // do timing test
      int iterations = 10;
      long start = System.currentTimeMillis();

      for (int i = 0; i < iterations; i++)
      {
         for (int j = 0; j < inputs.length; j++)
         {
            solver.initialize(inputs[j]);
         }
      }

      solver.solve();

      long stop = System.currentTimeMillis();

      long diff = stop - start;
      System.out.println("Solver time: " + (diff / (iterations * inputs.length)) + "ms") ;
   }

   public static void main(String[] args)
   {
      runTimingTest();
   }
}
