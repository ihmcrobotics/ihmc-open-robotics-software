package us.ihmc.utilities.ros.msgToPacket.converter;

import us.ihmc.utilities.math.trajectories.TrajectoryType;

public class TrajectoryWaypointGenerationMethodConverter
{
   public static TrajectoryType convertTrajectoryWaypointGenerationMethod(byte b)
   {
      return TrajectoryType.values()[(int) b];
   }
}
