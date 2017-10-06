package us.ihmc.robotics.trajectories.providers;

public class TrajectoryParametersProvider
{
   private TrajectoryParameters trajectoryParameters;

   public TrajectoryParametersProvider()
   {
      this(null);
   }
   
   public TrajectoryParametersProvider(TrajectoryParameters trajectoryParameters)
   {
      this.trajectoryParameters = trajectoryParameters;
   }
   
   public TrajectoryParameters getTrajectoryParameters()
   {
      return trajectoryParameters;
   }

   public void set(TrajectoryParameters trajectoryParameters)
   {
      this.trajectoryParameters = trajectoryParameters;
   }
}
