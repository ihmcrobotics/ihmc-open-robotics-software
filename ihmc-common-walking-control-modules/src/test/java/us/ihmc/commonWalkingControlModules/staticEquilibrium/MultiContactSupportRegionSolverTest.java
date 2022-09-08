package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;

import java.util.Arrays;
import java.util.List;

public class MultiContactSupportRegionSolverTest
{
   private static final boolean VISUALIZE = true; //  Boolean.parseBoolean(System.getProperty("visualize", "true"));
   private static final double marginToTest = 1e-3;

   @Test
   public void testStaticEquilibriumSolver()
   {
      List<MultiContactSupportRegionSolverInput> inputsToTest = Arrays.asList(MultiContactSupportRegionSolverInputExamples.createTriangleFlatGround(),
                                                                              MultiContactSupportRegionSolverInputExamples.createTriangleTiltedOutSlightly(),
                                                                              MultiContactSupportRegionSolverInputExamples.createTriangleTiltedOutALot(),
                                                                              MultiContactSupportRegionSolverInputExamples.createTriangleOneTiltedFullyOut(),
                                                                              MultiContactSupportRegionSolverInputExamples.createTriangleOneTiltedFullyIn(),
                                                                              MultiContactSupportRegionSolverInputExamples.createFlatSquare(),
                                                                              MultiContactSupportRegionSolverInputExamples.createBipedFeet(),
                                                                              MultiContactSupportRegionSolverInputExamples.createBipedFeetWithSingleHandhold(),
                                                                              MultiContactSupportRegionSolverInputExamples.createBipedFeetWithSingleHandhold());

      for (int i = 0; i < inputsToTest.size(); i++)
      {
         runTest(inputsToTest.get(i));
      }
   }

   private void runTest(MultiContactSupportRegionSolverInput input)
   {
      MultiContactSupportRegionSolver solver = new MultiContactSupportRegionSolver();
      solver.initialize(input);
      solver.solve();

      ConvexPolygon2DReadOnly supportPolygon = solver.getSupportRegion();
      MultiContactForceOptimizer forceOptimizer = new MultiContactForceOptimizer();

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
      MultiContactSupportRegionSolver solver = new MultiContactSupportRegionSolver();

      MultiContactSupportRegionSolverInput input0 = MultiContactSupportRegionSolverInputExamples.createTriangleTiltedOutSlightly();
      MultiContactSupportRegionSolverInput input1 = MultiContactSupportRegionSolverInputExamples.createTriangleOneTiltedFullyIn();
      MultiContactSupportRegionSolverInput input2 = MultiContactSupportRegionSolverInputExamples.createBipedFeet();
      MultiContactSupportRegionSolverInput input3 = MultiContactSupportRegionSolverInputExamples.createBipedFeetWithSingleHandhold();
      MultiContactSupportRegionSolverInput input4 = MultiContactSupportRegionSolverInputExamples.createBipedFeetWithTwoHandholds();

      MultiContactSupportRegionSolverInput[] inputs = new MultiContactSupportRegionSolverInput[]{input0, input1, input2, input3, input4};

      // warm up
      int warmups = 20;
      for (int i = 0; i < warmups; i++)
      {
         solver.initialize(input0);
         solver.solve();
      }

      // do timing test
      int iterations = 20;
      long startNano = System.nanoTime();

      for (int i = 0; i < iterations; i++)
      {
         for (int j = 0; j < inputs.length; j++)
         {
            solver.initialize(inputs[j]);
            solver.solve();
         }
      }

      long stopNano = System.nanoTime();

      double diffUs = (stopNano - startNano) * 1e-3;
      System.out.println("Average solve time: " + (diffUs / (iterations * inputs.length)) + " micro-sec") ;
   }

   public static void main(String[] args)
   {
      runTimingTest();
   }
}
