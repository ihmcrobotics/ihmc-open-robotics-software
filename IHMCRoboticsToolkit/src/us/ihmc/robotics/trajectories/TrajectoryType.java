package us.ihmc.robotics.trajectories;

public enum TrajectoryType
{
   DEFAULT, BASIC, PUSH_RECOVERY, OBSTACLE_CLEARANCE;

   public String getDocumentation(TrajectoryType var)
   {
      switch (var)
      {
      case DEFAULT:
         return "is a default trajectory";
      case BASIC:
         return "will do a basic swing with the specified swing height";
      case PUSH_RECOVERY:
         return "uses a low swing height for fast steps";
      case OBSTACLE_CLEARANCE:
         return "will attempt to step over an obstacle";
         
      default:
         return "no documentation available";
      }
   }

   public TrajectoryType[] getDocumentedValues()
   {
      return values();
   }
}