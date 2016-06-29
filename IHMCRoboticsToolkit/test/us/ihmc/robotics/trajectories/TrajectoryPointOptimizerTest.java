package us.ihmc.robotics.trajectories;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.waypoints.PolynomialOrder;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer;
import us.ihmc.tools.testing.MutationTestingTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class TrajectoryPointOptimizerTest
{
   private static final double epsilon = 10E-7;

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testEndPointSetters()
   {
      int dimensions = 3;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      YoVariableRegistry registry = new YoVariableRegistry("");
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order, registry);

      optimizer.setEndPoints(new double[dimensions], new double[dimensions], new double[dimensions], new double[dimensions]);
      optimizer.setEndPoints(new double[dimensions], new double[dimensions-1], new double[dimensions], new double[dimensions]);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000, expected=RuntimeException.class)
   public void testWaypointSetters()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      ArrayList<double[]> waypoints = new ArrayList<>();
      waypoints.add(new double[] {1.0});
      waypoints.add(new double[] {5.0});
      waypoints.add(new double[] {21.0});
      optimizer.setWaypoints(waypoints);

      waypoints.add(new double[] {5.0, 1.7});
      optimizer.setWaypoints(waypoints);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
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

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleSymmetricProblem()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      double x0 = 0.0;
      double xd0 = 0.0;
      double x1 = 1.0;
      double xd1 = 0.0;

      optimizer.setEndPoints(new double[] {x0}, new double[] {xd0}, new double[] {x1}, new double[] {xd1});

      // compute twice
      optimizer.compute();
      ArrayList<double[]> dummyWaypoints = new ArrayList<>();
      dummyWaypoints.add(new double[] {0.25});
      dummyWaypoints.add(new double[] {0.3});
      optimizer.setWaypoints(dummyWaypoints);
      optimizer.compute();

      ArrayList<double[]> waypoints = new ArrayList<>();
      waypoints.add(new double[] {0.5});
      optimizer.setWaypoints(waypoints);
      optimizer.compute();

      ArrayList<double[]> coefficients = new ArrayList<>();
      for (int i = 0; i < waypoints.size() + 1; i++)
         coefficients.add(new double[order.getCoefficients()]);
      optimizer.getPolynomialCoefficients(coefficients, 0);

      // computed by hand:
      double[] expected = new double[] {-2.0, 3.0, 0.0, 0.0};

      double[] waypointTimes = new double[waypoints.size()];
      optimizer.getWaypointTimes(waypointTimes);
//      printResults(waypointTimes, coefficients);

      for (int i = 0; i < order.getCoefficients(); i++)
      {
         assertEquals(coefficients.get(0)[i], expected[i], epsilon);
         assertEquals(coefficients.get(0)[i], coefficients.get(1)[i], epsilon);
      }
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleProblem()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER3;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      double x0 = 1.0;
      double xd0 = 1.0;
      double x1 = 2.0;
      double xd1 = 1.0;

      optimizer.setEndPoints(new double[] {x0}, new double[] {xd0}, new double[] {x1}, new double[] {xd1});
      optimizer.compute();

      ArrayList<double[]> coefficients = new ArrayList<>();
      coefficients.add(new double[order.getCoefficients()]);
      optimizer.getPolynomialCoefficients(coefficients, 0);

      double[] expected = new double[] {0.0, 0.0, 1.0, 1.0};
      for (int i = 0; i < order.getCoefficients(); i++)
         assertEquals(coefficients.get(0)[i], expected[i], epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrivialProblem()
   {
      int dimensions = 1;
      PolynomialOrder order = PolynomialOrder.ORDER7;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      double x0 = 0.0;
      double xd0 = 1.0;
      double x1 = 1.0;
      double xd1 = 1.0;

      optimizer.setEndPoints(new double[] {x0}, new double[] {xd0}, new double[] {x1}, new double[] {xd1});
      optimizer.compute();

      ArrayList<double[]> coefficients = new ArrayList<>();
      coefficients.add(new double[order.getCoefficients()]);
      optimizer.getPolynomialCoefficients(coefficients, 0);

      double[] expected = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0};
      for (int i = 0; i < order.getCoefficients(); i++)
         assertEquals(expected[i], coefficients.get(0)[i], epsilon);
   }

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTimeDescent()
   {
      int dimensions = 2;
      PolynomialOrder order = PolynomialOrder.ORDER5;
      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      double[] x0 = new double[] {0.0, 0.0};
      double[] xd0 = new double[] {0.0, 0.0};
      double[] x1 = new double[] {1.0, 1.0};
      double[] xd1 = new double[] {0.0, 0.0};

      optimizer.setEndPoints(x0, xd0, x1, xd1);
      ArrayList<double[]> waypoints = new ArrayList<>();
      waypoints.add(new double[] {0.001, 0.001});
      waypoints.add(new double[] {0.8, 0.8});
      optimizer.setWaypoints(waypoints);

      optimizer.compute();

      double[] waypointTimes = new double[waypoints.size()];
      optimizer.getWaypointTimes(waypointTimes);
//      printResults(waypointTimes, new ArrayList<>());

      assertTrue(waypointTimes[0] < 0.1);
      assertTrue(waypointTimes[0] > 0.0);
   }

   private void printResults(double[] waypointTimes, ArrayList<double[]> coefficients)
   {
      String timesString = "";
      for (int i = 0; i < waypointTimes.length; i++)
      {
         timesString += waypointTimes[i] + " ";
      }
      System.out.println("Waypoint Times: " + timesString);
      for (double[] coeffs : coefficients)
      {
         String coeffString = "";
         for (int i = 0; i < coeffs.length; i++)
         {
            coeffString += coeffs[i] + " ";
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
