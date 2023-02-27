package us.ihmc.robotics.math.trajectories.waypoints;

import java.lang.reflect.Method;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.EuclideanWaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointReadOnly;

public class FrameEuclideanWaypointTest
{
   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(TrajectoryPointRandomTools::nextFrameEuclideanWaypoint,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithEuclideanWaypoint() throws Exception
   {
      FrameTypeCopier frameTypeBuilder = (frame, waypoint) -> new FrameEuclideanWaypoint(frame, (EuclideanWaypointReadOnly) waypoint);
      RandomFramelessTypeBuilder framelessTypeBuilber = TrajectoryPointRandomTools::nextEuclideanWaypoint;
      Predicate<Method> methodFilter = m -> !m.getName().equals("hashCode") && !m.getName().equals("toString");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertFrameMethodsOfFrameHolderPreserveFunctionality(frameTypeBuilder,
                                                                  framelessTypeBuilber,
                                                                  methodFilter,
                                                                  EuclidTestConstants.API_FUNCTIONALITY_TEST_ITERATIONS);
   }

   @Test
   public void testOverloading() throws Exception
   {
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertOverloadingWithFrameObjects(FrameEuclideanWaypointBasics.class, EuclideanWaypointBasics.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameEuclideanWaypointReadOnly.class, EuclideanWaypointReadOnly.class, false, 1);
   }
}
