package us.ihmc.robotics.math.trajectories;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;


public class IntermediateWaypointVelocityGeneratorTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test()
   {
      final ArrayList<Vector3D> waypoints;
      final ArrayList<Double> time;

      waypoints = new ArrayList<>(6);
      time = new ArrayList<>();

      waypoints.add(new Vector3D(0.00, 0.00, 0.00));
      waypoints.add(new Vector3D(1.00, 2.00, 3.00));
      waypoints.add(new Vector3D(5.00, 5.00, 5.00));
      waypoints.add(new Vector3D(4.00, 3.00, 9.00));
      waypoints.add(new Vector3D(2.00, 4.00, 6.00));
      waypoints.add(new Vector3D(0.00, 0.00, 0.00));

      time.add(1.0);
      time.add(2.3);
      time.add(2.9);
      time.add(3.5);
      time.add(4.3);
      time.add(6.0);

      IntermediateWaypointVelocityGenerator velocityGen = new IntermediateWaypointVelocityGenerator(time, waypoints);
      velocityGen.calculateVelocities();
      velocityGen.printResultsForTest();
      System.out.println("end");

   }
}
