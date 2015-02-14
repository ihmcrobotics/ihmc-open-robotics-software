package us.ihmc.utilities.ros.msgToPacket.converter;

import us.ihmc.utilities.math.trajectories.TrajectoryGenerationMethod;

public class TrajectoryWaypointGenerationMethodConverter
{
   public static TrajectoryGenerationMethod convertTrajectoryWaypointGenerationMethod(byte b)
   {
      return TrajectoryGenerationMethod.values()[(int) b];
   }
}
