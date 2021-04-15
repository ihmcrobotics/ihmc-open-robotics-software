package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

public class StaticEquilibriumSolverTest
{
   private static final boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize", "true"));
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
      runTest(StaticEquilibriumSolverInputExamples.createBipedFeetWithHandhold());
   }

   private void runTest(StaticEquilibriumSolverInput input)
   {
      StaticEquilibriumSolver solver = new StaticEquilibriumSolver();
      solver.solve(input);

      ConvexPolygon2D supportPolygon = new ConvexPolygon2D();
      solver.getSupportRegion().forEach(supportPolygon::addVertex);
      supportPolygon.update();

      StaticEquilibriumForceOptimizer forceOptimizer = new StaticEquilibriumForceOptimizer();

      // test the vertices are valid
      for (int i = 0; i < supportPolygon.getNumberOfVertices(); i++)
      {
         Point2DReadOnly vertex = supportPolygon.getVertex(i);
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
         boolean succeeded = forceOptimizer.solve(input, vertex);
         Assertions.assertFalse(succeeded);
//         System.out.println(!succeeded ? "pass" : "fail");
      }
   }
}
