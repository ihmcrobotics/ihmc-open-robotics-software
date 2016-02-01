package us.ihmc.utilities.ros.msgToPacket.converter;

import us.ihmc.robotics.trajectories.TrajectoryType;

public class TrajectoryWaypointGenerationMethodConverter
{
   public static TrajectoryType convertTrajectoryWaypointGenerationMethod(byte b)
   {
      return TrajectoryType.values()[(int) b];
   }
}
