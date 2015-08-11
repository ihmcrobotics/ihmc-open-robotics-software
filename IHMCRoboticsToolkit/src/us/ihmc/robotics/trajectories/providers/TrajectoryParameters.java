package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.trajectories.TrajectoryType;

public class TrajectoryParameters
{
   private TrajectoryType trajectoryType;
   private double swingHeight = 0;

   public TrajectoryParameters(){
      this(null, 0.0);
   }

   public TrajectoryParameters(double swingHeight){
      this(null, swingHeight);
   }

   public TrajectoryParameters(TrajectoryType trajectoryType){
      this(trajectoryType, 0.0);
   }

   public TrajectoryParameters(TrajectoryType trajectoryType, double swingHeight){
      if (trajectoryType == null)
      {
         this.trajectoryType = TrajectoryType.DEFAULT;
      }else{
         this.trajectoryType = trajectoryType;
      }
      this.swingHeight = swingHeight;
   }

   public TrajectoryType getTrajectoryType()
   {
      return trajectoryType;
   }

   public double getSwingHeight()
   {
      return swingHeight;
   }

}