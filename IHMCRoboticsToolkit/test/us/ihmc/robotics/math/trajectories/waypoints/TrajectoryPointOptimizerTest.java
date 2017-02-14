package us.ihmc.robotics.math.trajectories.waypoints;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.tools.testing.MutationTestingTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class TrajectoryPointOptimizerTest
{
   private static final double epsilon = 10E-7;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testEndPointSetters()
   {
      int dimensions = 3;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      YoVariableRegistry registry = new YoVariableRegistry("");
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order, registry);

      TDoubleArrayList rightSize = new TDoubleArrayList(dimensions);
      TDoubleArrayList wrongSize = new TDoubleArrayList(dimensions-1);

      for (int i = 0; i < dimensions; i++)
         rightSize.add(0.0);

      optimizer.setEndPoints(rightSize, rightSize, rightSize, rightSize);
      optimizer.setEndPoints(rightSize, wrongSize, rightSize, rightSize);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testWaypointSetters()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

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
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testYoVariables()
   {
      int dimensions = 3;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      YoVariableRegistry registry = new YoVariableRegistry("");
      new TrajectoryPointOptimizer(dimensions, order, registry);

      DoubleYoVariable timeGain = (DoubleYoVariable) registry.getVariable("TimeGain");
      assertTrue(timeGain.getDoubleValue() != 0.0);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleSymmetricProblem()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

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
         coefficients.add(new TDoubleArrayList(order.getCoefficients()));
      optimizer.getPolynomialCoefficients(coefficients, 0);

      // computed by hand:
      double[] expected = new double[] {-2.0, 3.0, 0.0, 0.0};

      TDoubleArrayList waypointTimes = new TDoubleArrayList(waypoints.size());
      optimizer.getWaypointTimes(waypointTimes);
//      printResults(waypointTimes, coefficients);

      for (int i = 0; i < order.getCoefficients(); i++)
      {
         assertEquals(coefficients.get(0).get(i), expected[i], epsilon);
         assertEquals(coefficients.get(0).get(i), coefficients.get(1).get(i), epsilon);
      }

      TDoubleArrayList waypointVelocity = new TDoubleArrayList();
      optimizer.getWaypointVelocity(waypointVelocity, 0);
      assertEquals(waypointVelocity.get(0), 1.5, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleProblem()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

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
      for (int i = 0; i < order.getCoefficients(); i++)
         assertEquals(coefficients.get(0).get(i), expected[i], epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrivialProblem()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER5;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      TDoubleArrayList x0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd0 = new TDoubleArrayList(dimensions);
      TDoubleArrayList x1 = new TDoubleArrayList(dimensions);
      TDoubleArrayList xd1 = new TDoubleArrayList(dimensions);

      x0.add(0.0);
      xd0.add(1.0);
      x1.add(1.0);
      xd1.add(1.0);

      optimizer.setEndPoints(x0, xd0, x1, xd1);
      optimizer.compute();

      ArrayList<TDoubleArrayList> coefficients = new ArrayList<>();
      coefficients.add(new TDoubleArrayList(0));
      optimizer.getPolynomialCoefficients(coefficients, 0);

      double[] expected = new double[] {0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
      for (int i = 0; i < order.getCoefficients(); i++)
         assertEquals(expected[i], coefficients.get(0).get(i), epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTimeDescent()
   {
      int dimensions = 2;
      PolynomialOrder order = PolynomialOrder.ORDER5;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

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
      String targetTests = "us.ihmc.robotics.trajectories.TrajectoryPointOptimizerTest";
      String targetClasses = "us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer";
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClasses);
   }
}
