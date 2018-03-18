package us.ihmc.commonWalkingControlModules.centroidalMotionPlanner;

import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class CentroidalMotionPlannerTools
{
   // Hidden constructor to prevent creation of objects
   private CentroidalMotionPlannerTools()
   {
      
   }
   
   public static GenericTypeBuilder<Trajectory> getTrajectoryBuilder(int maxNumberOfCoefficients)
   {
      return new GenericTypeBuilder<Trajectory>()
      {
         @Override
         public Trajectory newInstance()
         {
            return new Trajectory(maxNumberOfCoefficients);
         }
      };
   }
}
