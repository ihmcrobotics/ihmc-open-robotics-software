package us.ihmc.robotics.math.trajectories.waypoints;

import java.lang.reflect.Method;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SE3WaypointReadOnly;

public class FrameSE3WaypointTest
{
   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(TrajectoryPointRandomTools::nextFrameSE3Waypoint,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithSE3Waypoint() throws Exception
   {
      FrameTypeCopier frameTypeBuilder = (frame, waypoint) -> new FrameSE3Waypoint(frame, (SE3WaypointReadOnly) waypoint);
      RandomFramelessTypeBuilder framelessTypeBuilber = TrajectoryPointRandomTools::nextSE3Waypoint;
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
      tester.assertOverloadingWithFrameObjects(FrameSE3WaypointBasics.class, SE3WaypointBasics.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameSE3WaypointReadOnly.class, SE3WaypointReadOnly.class, false, 1);
   }
}
