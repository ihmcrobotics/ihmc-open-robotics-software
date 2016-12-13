package us.ihmc.robotics.trajectories;

public enum TrajectoryType
{
   DEFAULT, OBSTACLE_CLEARANCE;

   public static String getDocumentation(TrajectoryType var)
   {
      switch (var)
      {
      case DEFAULT:
         return "is a default trajectory";
      case OBSTACLE_CLEARANCE:
         return "will attempt to step over an obstacle";

      default:
         return "no documentation available";
      }
   }
}