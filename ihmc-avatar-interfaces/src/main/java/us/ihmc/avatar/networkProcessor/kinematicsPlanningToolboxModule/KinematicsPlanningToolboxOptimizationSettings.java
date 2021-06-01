package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

public class KinematicsPlanningToolboxOptimizationSettings implements SolutionQualityConvergenceSettings
{
   @Override
   public double getSolutionQualityThreshold()
   {
      return 0.05;
   }

   @Override
   public double getSolutionStabilityThreshold()
   {
      return 0.05;
   }
   
   @Override
   public int getDefaultTerminalIteration()
   {
      return 30;
   }
}
