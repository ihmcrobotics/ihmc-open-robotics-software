package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

public class KinematicsPlanningToolboxOptimizationSettings implements SolutionQualityConvergenceSettings
{
   @Override
   public double getSolutionStabilityThreshold()
   {
      return 0.001;
   }
   
   @Override
   public int getDefaultTerminalIteration()
   {
      return 20;
   }
}
