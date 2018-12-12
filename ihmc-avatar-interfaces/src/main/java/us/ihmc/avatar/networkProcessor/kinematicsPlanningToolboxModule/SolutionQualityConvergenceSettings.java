package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

public interface SolutionQualityConvergenceSettings
{
   default boolean useConvergenceDetecting()
   {
      return true;
   }

   default double getSolutionQualityThreshold()
   {
      return 0.005;
   }

   default double getSolutionStabilityThreshold()
   {
      return 0.00002;
   }

   default double getMinimumProgression()
   {
      return 0.005;
   }

   default int getDefaultTerminalIteration()
   {
      return 50;
   }
}
