package us.ihmc.utilities.ros.msgToPacket.converter;

import us.ihmc.utilities.math.trajectories.TrajectoryWaypointGenerationMethod;

public class TrajectoryWaypointGenerationMethodConverter
{
   public static TrajectoryWaypointGenerationMethod convertTrajectoryWaypointGenerationMethod(byte b)
   {
      return TrajectoryWaypointGenerationMethod.values()[(int) b];
   }
}
