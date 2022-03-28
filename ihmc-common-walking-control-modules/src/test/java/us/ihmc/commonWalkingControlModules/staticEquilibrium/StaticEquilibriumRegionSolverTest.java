package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

import java.util.Arrays;
import java.util.List;

public class StaticEquilibriumRegionSolverTest
{
   private static final boolean VISUALIZE = false; //  Boolean.parseBoolean(System.getProperty("visualize", "true"));
   private static final double marginToTest = 1e-3;

   @Test
   public void testStaticEquilibriumSolver()
   {
      List<StaticEquilibriumSolverInput> inputsToTest = Arrays.asList(StaticEquilibriumSolverInputExamples.createTriangleFlatGround(),
                                                                      StaticEquilibriumSolverInputExamples.createTriangleTiltedOutSlightly(),
                                                                      StaticEquilibriumSolverInputExamples.createTriangleTiltedOutALot(),
                                                                      StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyOut(),
                                                                      StaticEquilibriumSolverInputExamples.createTriangleOneTiltedFullyIn(),
                                                                      StaticEquilibriumSolverInputExamples.createFlatSquare(),
                                                                      StaticEquilibriumSolverInputExamples.createBipedFeet(),
                                                                      StaticEquilibriumSolverInputExamples.createBipedFeetWithSingleHandhold(),
                                                                      StaticEquilibriumSolverInputExamples.createBipedFeetWithSingleHandhold());

      for (int i = 0; i < inputsToTest.size(); i++)
      {
         runTest(inputsToTest.get(i));
      }
   }

   private void runTest(StaticEquilibriumSolverInput input)
   {
      StaticSupportRegionSolver solver = new StaticSupportRegionSolver();
      solver.initialize(input);
      solver.solve();

      ConvexPolygon2DReadOnly supportPolygon = solver.getSupportRegion();
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
      int warmups = 20;
      for (int i = 0; i < warmups; i++)
      {
         solver.initialize(input0);
         solver.solve();
      }

      // do timing test
      int iterations = 20;
      long start = System.currentTimeMillis();

      for (int i = 0; i < iterations; i++)
      {
         for (int j = 0; j < inputs.length; j++)
         {
            solver.initialize(inputs[j]);
            solver.solve();
         }
      }

      long stop = System.currentTimeMillis();

      long diff = stop - start;
      System.out.println("Average solve time: " + (diff / (iterations * inputs.length)) + "ms") ;
   }

   public static void main(String[] args)
   {
      runTimingTest();
   }
}
