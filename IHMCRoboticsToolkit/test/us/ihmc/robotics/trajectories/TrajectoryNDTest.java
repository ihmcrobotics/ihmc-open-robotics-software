package us.ihmc.robotics.trajectories;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Assert;
import org.junit.Test;
import us.ihmc.robotics.trajectories.TrajectoryND.WaypointND;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.PrintWriter;

@DeployableTestClass(targets = TestPlanTarget.InDevelopment)
public class TrajectoryNDTest
{
   static private final boolean DEBUG = true;
//   static private final double dT = 0.001;
//   static private final double EPS = 0.0001;

	@DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void test()
   {
	   Assert.fail("Not yet implemented");
   }

   static public void printExampleTrajectory()
   {
      try
      {
         double maxVel = 10;
         double maxAcc = 55;
         TrajectoryND trajectory = new TrajectoryND(3, maxVel, maxAcc);

         trajectory.addWaypoint(new double[] {0, 0, 0});
         trajectory.addWaypoint(new double[] {2, 5, 2});
         trajectory.addWaypoint(new double[] {-2, 8, 4});
         trajectory.addWaypoint(new double[] {5, 0, 6});
         trajectory.addWaypoint(new double[] {0, 0, 0});

         if (DEBUG)
         {
            ImmutablePair<Boolean, WaypointND> results = trajectory.getNextInterpolatedPoints(0.01);

            PrintWriter fileout = new PrintWriter(new BufferedWriter(new FileWriter("testResources/us/ihmc/robotics/trajectory/multi_poly_test.txt", false)));

            while (results.getLeft().booleanValue() == false)
            {
               String content = new String();
               WaypointND wp = results.getRight();

               for (int d = 0; d < trajectory.getNumDimensions(); d++)
               {
                  if (d == 0)
                  {
                     content += wp.absTime;
                  }

                  content += " " + wp.position[d] + " " + wp.velocity[d] + " " + wp.acceleration[d];
               }

               content += "\n";

               fileout.print(content);


               System.out.print(content);

               Thread.sleep(20);

               results = trajectory.getNextInterpolatedPoints(0.01);
            }

            fileout.close();
            System.out.print("Done");

         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
