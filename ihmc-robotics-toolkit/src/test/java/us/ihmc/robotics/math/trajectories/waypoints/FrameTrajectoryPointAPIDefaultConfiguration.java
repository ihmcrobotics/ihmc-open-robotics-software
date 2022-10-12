package us.ihmc.robotics.math.trajectories.waypoints;

import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPIDefaultConfiguration;
import us.ihmc.euclid.referenceFrame.api.EuclidFrameAPITester;
import us.ihmc.euclid.referenceFrame.api.ReflectionBasedBuilder;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameEuclideanTrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.FrameSO3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameEuclideanWaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSE3WaypointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.interfaces.FrameSO3WaypointBasics;

public class FrameTrajectoryPointAPIDefaultConfiguration extends EuclidFrameAPIDefaultConfiguration
{
   @Override
   public void configure(EuclidFrameAPITester testerToConfigure, ReflectionBasedBuilder builderToConfigure)
   {
      super.configure(testerToConfigure, builderToConfigure);

      builderToConfigure.registerRandomGeneratorClasses(TrajectoryPointRandomTools.class);

      testerToConfigure.registerFrameTypesSmart(FrameEuclideanWaypointBasics.class, FrameSO3WaypointBasics.class, FrameSE3WaypointBasics.class);
      testerToConfigure.registerFrameTypesSmart(FrameEuclideanTrajectoryPointBasics.class,
                                                FrameSO3TrajectoryPointBasics.class,
                                                FrameSE3TrajectoryPointBasics.class);
   }
}
