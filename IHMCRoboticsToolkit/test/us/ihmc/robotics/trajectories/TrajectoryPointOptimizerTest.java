package us.ihmc.robotics.trajectories;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.robotics.math.trajectories.waypoints.PolynomialOrder;
import us.ihmc.robotics.math.trajectories.waypoints.TrajectoryPointOptimizer;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class TrajectoryPointOptimizerTest
{
   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test()
   {
      int dimensions = 2;
      PolynomialOrder order = PolynomialOrder.ORDER5;

      TrajectoryPointOptimizer optimizer = new TrajectoryPointOptimizer(dimensions, order);

      double[] x0 = {0.0, 0.0};
      double[] xd0 = {0.0, 0.0};
      double[] x1 = {1.0, 0.0};
      double[] xd1 = {0.0, 0.0};
      optimizer.setEndPoints(x0, xd0, x1, xd1);

      ArrayList<double[]> waypoints = new ArrayList<>();
      waypoints.add(new double[] {0.1, 0.1});
      waypoints.add(new double[] {0.9, 0.1});
      optimizer.setWaypoints(waypoints);

      optimizer.compute();

      double[] waypointTimes = new double[waypoints.size()];
      optimizer.getWaypointTimes(waypointTimes);
      ArrayList<double[]> coefficients = new ArrayList<>();
      for (int i = 0; i < waypoints.size() + 1; i++)
      {
         coefficients.add(new double[order.getCoefficients()]);
      }
      optimizer.getPolynomialCoefficients(coefficients, 0);

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
}
