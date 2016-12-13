package us.ihmc.robotics.trajectories;

public enum TrajectoryType
{
   DEFAULT, OBSTACLE_CLEARANCE, CUSTOM;

   public final static TrajectoryType[] values = values();

   public static String getDocumentation(TrajectoryType var)
   {
      switch (var)
      {
      case DEFAULT:
         return "is a default trajectory";
      case OBSTACLE_CLEARANCE:
         return "will attempt to step over an obstacle";
      case CUSTOM:
         return "allows to specify trajectory waypoints";

      default:
         return "no documentation available";
      }
   }
}