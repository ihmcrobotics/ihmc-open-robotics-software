package us.ihmc.robotics.math.trajectories.waypoints;

import static us.ihmc.robotics.Assert.assertArrayEquals;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.robotics.math.trajectories.generators.TrajectoryPointOptimizer;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TrajectoryPointOptimizerTest
{
   private static final double epsilon = 10E-7;

   @Test
   public void testEndPointSetters()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      int dimensions = 3;
      YoRegistry registry = new YoRegistry("Dummy");
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, registry);

      TDoubleArrayList rightSize = new TDoubleArrayList(dimensions);
      TDoubleArrayList wrongSize = new TDoubleArrayList(dimensions-1);

      for (int i = 0; i < dimensions; i++)
         rightSize.add(0.0);

      optimizer.setEndPoints(rightSize, rightSize, rightSize, rightSize);
      optimizer.setEndPoints(rightSize, wrongSize, rightSize, rightSize);
      });
   }

   @Test
   public void testWaypointSetters()
   {
      Assertions.assertThrows(RuntimeException.class, () -> {
      int dimensions = 1;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions);

      ArrayList<TDoubleArrayList> waypoints = new ArrayList<>();
      TDoubleArrayList waypointA = new TDoubleArrayList();
      TDoubleArrayList waypointB = new TDoubleArrayList();
      TDoubleArrayList waypointC = new TDoubleArrayList();
      waypointA.add(1.0);
      waypointB.add(5.0);
      waypointC.add(21.0);

      waypoints.add(waypointA);
      waypoints.add(waypointB);
      waypoints.add(waypointC);
      optimizer.setWaypoints(waypoints);

      TDoubleArrayList waypointD = new TDoubleArrayList();
      waypointD.add(5.0);
      waypointD.add(1.7);
      waypoints.add(waypointD);
      optimizer.setWaypoints(waypoints);
      });
   }

   @Test
   public void testYoVariables()
   {
      int dimensions = 3;
      YoRegistry registry = new YoRegistry("Dummy");
      new TrajectoryPointOptimizer(dimensions, registry);

      YoDouble timeGain = (YoDouble) registry.findVariable("TimeGain");
      assertTrue(timeGain.getDoubleValue() != 0.0);

   }

   @Test
   public void testSimpleSymmetricProblem()
   {
      int dimensions = 1;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions);

      TDoubleArrayList x0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList x1 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd1 = new TDoubleArrayList(dimensions);

      for (int i = 0; i < dimensions; i++)
      {
         x0.add(0.0);
         xd0.add(0.0);
         x1.add(0.0);
         xd1.add(0.0);
      }

      x1.set(0, 1.0);

      optimizer.setEndPoints(x0, xd0, x1, xd1);

      // compute twice
      optimizer.compute();
      ArrayList<TDoubleArrayList> dummyWaypoints = new ArrayList<>();
      TDoubleArrayList waypointA = new TDoubleArrayList();
      TDoubleArrayList waypointB = new TDoubleArrayList();
      waypointA.add(0.25);
      waypointB.add(0.3);

      dummyWaypoints.add(waypointA);
      dummyWaypoints.add(waypointB);
      optimizer.setWaypoints(dummyWaypoints);
      optimizer.compute();

      ArrayList<TDoubleArrayList> waypoints = new ArrayList<>();
      TDoubleArrayList waypointC = new TDoubleArrayList();
      waypointC.add(0.5);

      waypoints.add(waypointC);
      optimizer.setWaypoints(waypoints);
      optimizer.compute();

      ArrayList<TDoubleArrayList> coefficients = new ArrayList<>();
      for (int i = 0; i < waypoints.size() + 1; i++)
         coefficients.add(new TDoubleArrayList(TrajectoryPointOptimizer.coefficients));
      optimizer.getPolynomialCoefficients(coefficients, 0);

      // computed by hand:
      double[] expected = new double[] {-2.0, 3.0, 0.0, 0.0};

      TDoubleArrayList waypointTimes = new TDoubleArrayList(waypoints.size());
      optimizer.getWaypointTimes(waypointTimes);
      printResults(waypointTimes, coefficients);

      for (int i = 0; i < TrajectoryPointOptimizer.coefficients; i++)
      {
         assertEquals(coefficients.get(0).get(i), expected[i], epsilon);
         assertEquals(coefficients.get(0).get(i), coefficients.get(1).get(i), epsilon);
      }

      TDoubleArrayList waypointVelocity = new TDoubleArrayList();
      optimizer.getWaypointVelocity(waypointVelocity, 0);
      assertEquals(waypointVelocity.get(0), 1.5, epsilon);
   }

   @Test
   public void testSimpleProblem()
   {
      int dimensions = 1;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions);

      TDoubleArrayList x0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList x1 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd1 = new TDoubleArrayList(dimensions);

      x0.add(1.0);
      xd0.add(1.0);
      x1.add(2.0);
      xd1.add(1.0);

      optimizer.setEndPoints(x0, xd0, x1, xd1);
      optimizer.compute();

      ArrayList<TDoubleArrayList> coefficients = new ArrayList<>();
      coefficients.add(new TDoubleArrayList());
      optimizer.getPolynomialCoefficients(coefficients, 0);

      double[] expected = new double[] {0.0, 0.0, 1.0, 1.0};
      for (int i = 0; i < TrajectoryPointOptimizer.coefficients; i++)
         assertEquals(coefficients.get(0).get(i), expected[i], epsilon);
   }

   @Test
   public void testTrivialProblem()
   {
      int dimensions = 1;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions);

      TDoubleArrayList x0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList x1 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd1 = new TDoubleArrayList(dimensions);

      x0.add(0.0);
      xd0.add(1.0);
      x1.add(1.0);
      xd1.add(1.0);

      optimizer.setEndPoints(x0, xd0, x1, xd1);
      optimizer.compute(0);

      ArrayList<TDoubleArrayList> coefficients = new ArrayList<>();
      coefficients.add(new TDoubleArrayList(0));
      optimizer.getPolynomialCoefficients(coefficients, 0);

      double[] expected = new double[] {0.0, 0.0, 1.0, 0.0};
      for (int i = 0; i < TrajectoryPointOptimizer.coefficients; i++)
         assertEquals(expected[i], coefficients.get(0).get(i), epsilon);
   }

   @Test
   public void testTimeDescent()
   {
      int dimensions = 2;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions);

      TDoubleArrayList x0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList x1 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd1 = new TDoubleArrayList(dimensions);

      for (int i = 0; i < dimensions; i++)
      {
         x0.add(0.0);
         xd0.add(0.0);
         x1.add(0.0);
         xd1.add(0.0);
      }

      x1.set(0, 1.0);
      x1.set(1, 1.0);

      optimizer.setEndPoints(x0, xd0, x1, xd1);
      ArrayList<TDoubleArrayList> waypoints = new ArrayList<>();
      TDoubleArrayList waypointA = new TDoubleArrayList();
      TDoubleArrayList waypointB = new TDoubleArrayList();
      waypointA.add(0.001);
      waypointA.add(0.001);
      waypointB.add(0.8);
      waypointB.add(0.8);

      waypoints.add(waypointA);
      waypoints.add(waypointB);
      optimizer.setWaypoints(waypoints);

      optimizer.compute();

      TDoubleArrayList waypointTimes = new TDoubleArrayList();
      optimizer.getWaypointTimes(waypointTimes);
//      printResults(waypointTimes, new ArrayList<>());

      assertTrue(waypointTimes.get(0) < 0.1);
      assertTrue(waypointTimes.get(0) > 0.0);
   }

   @Test
   public void testGetEndpoints()
   {
      int dimensions = 3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions);

      TDoubleArrayList input_x0 = new TDoubleArrayList(new double[] {0.654, 0.129, -0.004});
      TDoubleArrayList input_xd0 = new TDoubleArrayList(new double[] {0.045, 0.015, 0.207});
      TDoubleArrayList input_x1 = new TDoubleArrayList(new double[] {1.416, 0.125, -0.022});
      TDoubleArrayList input_xd1 = new TDoubleArrayList(new double[] {0.000, 0.000, -0.300});
      List<TDoubleArrayList> input_waypoints = new ArrayList<>();
      input_waypoints.add(new TDoubleArrayList(new double[] {0.768, 0.129, 0.093}));
      input_waypoints.add(new TDoubleArrayList(new double[] {1.301, 0.126, 0.081}));

      optimizer.setEndPoints(input_x0, input_xd0, input_x1, input_xd1);
      optimizer.setWaypoints(input_waypoints);
      optimizer.compute();

      
      TDoubleArrayList output_x0 = new TDoubleArrayList();
      TDoubleArrayList output_xd0 = new TDoubleArrayList();
      TDoubleArrayList output_x1 = new TDoubleArrayList();
      TDoubleArrayList output_xd1 = new TDoubleArrayList();
      optimizer.getStartPosition(output_x0);
      optimizer.getStartVelocity(output_xd0);
      optimizer.getTargetPosition(output_x1);
      optimizer.getTargetVelocity(output_xd1);

      assertArrayEquals(input_x0.toArray(), output_x0.toArray(), epsilon);
      assertArrayEquals(input_xd0.toArray(), output_xd0.toArray(), epsilon);
      assertArrayEquals(input_x1.toArray(), output_x1.toArray(), epsilon);
      assertArrayEquals(input_xd1.toArray(), output_xd1.toArray(), epsilon);
   }

   private void printResults(TDoubleArrayList waypointTimes, ArrayList<TDoubleArrayList> coefficients)
   {
      String timesString = "";
      for (int i = 0; i < waypointTimes.size(); i++)
      {
         timesString += waypointTimes.get(i) + " ";
      }
      System.out.println("Waypoint Times: " + timesString);
      for (TDoubleArrayList coeffs : coefficients)
      {
         String coeffString = "";
         for (int i = 0; i < coeffs.size(); i++)
         {
            coeffString += coeffs.get(i) + " ";
         }
         System.out.println("Polynomial Coefficients: " + coeffString);
      }
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(TrajectoryPointOptimizer.class, TrajectoryPointOptimizerTest.class);
   }
}
