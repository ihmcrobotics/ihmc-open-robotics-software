package us.ihmc.robotics.trajectories;

public enum TrajectoryType
{
   DEFAULT, OBSTACLE_CLEARANCE, CUSTOM, WAYPOINTS;

   public final static TrajectoryType[] values = values();

   public static String getDocumentation(TrajectoryType var)
   {
      switch (var)
      {
      case DEFAULT:
         return "The controller will execute a default trajectory.";
      case OBSTACLE_CLEARANCE:
         return "The controller will attempt to step on/off an obstacle.";
      case CUSTOM:
         return "In this mode two trajectory position waypoints can be specified.";
      case WAYPOINTS:
         return "The swing trajectory is fully defined by the given waypoints.";

      default:
         throw new RuntimeException("Document this case.");
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static TrajectoryType fromByte(byte enumAsByte)
   {
      if (enumAsByte == -1)
         return null;
      return values[enumAsByte];
   }
}