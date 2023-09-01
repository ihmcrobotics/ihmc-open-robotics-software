package us.ihmc.robotics.math.trajectories.waypoints;

import java.lang.reflect.Method;
import java.util.function.Predicate;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.EuclidTestConstants;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.FrameTypeCopier;
import us.ihmc.euclid.referenceFrame.api.RandomFramelessTypeBuilder;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointReadOnly;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.SO3WaypointReadOnly;

public class FrameSO3WaypointTest
{
   @Test
   public void testReferenceFrameChecks() throws Throwable
   {
      Predicate<Method> methodFilter = m -> !m.getName().equals("equals") && !m.getName().equals("epsilonEquals");
      EuclidFrameAPITester tester = new EuclidFrameAPITester(new FrameTrajectoryPointAPIDefaultConfiguration());
      tester.assertMethodsOfReferenceFrameHolderCheckReferenceFrame(TrajectoryPointRandomTools::nextFrameSO3Waypoint,
                                                                    methodFilter,
                                                                    EuclidTestConstants.API_FRAME_CHECKS_ITERATIONS);
   }

   @Test
   public void testConsistencyWithSO3Waypoint() throws Exception
   {
      FrameTypeCopier frameTypeBuilder = (frame, waypoint) -> new FrameSO3Waypoint(frame, (SO3WaypointReadOnly) waypoint);
      RandomFramelessTypeBuilder framelessTypeBuilber = TrajectoryPointRandomTools::nextSO3Waypoint;
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
      tester.assertOverloadingWithFrameObjects(FrameSO3WaypointBasics.class, SO3WaypointBasics.class, false, 1);
      tester.assertOverloadingWithFrameObjects(FrameSO3WaypointReadOnly.class, SO3WaypointReadOnly.class, false, 1);
   }
}
