package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SolutionQualityConvergenceDetector
{
   private final YoDouble solutionQuality;
   private final YoDouble solutionQualityThreshold;
   private final YoDouble solutionStabilityThreshold;
   private final YoDouble solutionMinimumProgression;

   private final YoInteger numberOfIterations;
   private final YoInteger maximumNumberOfIterations;

   private final YoDouble computationTime;
   private final YoBoolean isConverged;

   public SolutionQualityConvergenceDetector(SolutionQualityConvergenceSettings settings, YoVariableRegistry parentRegistry)
   {
      solutionQuality = new YoDouble("solutionQuality", parentRegistry);
      solutionQualityThreshold = new YoDouble("solutionQualityThreshold", parentRegistry);
      solutionStabilityThreshold = new YoDouble("solutionStabilityThreshold", parentRegistry);
      solutionMinimumProgression = new YoDouble("solutionProgressionThreshold", parentRegistry);

      numberOfIterations = new YoInteger("convergenceDetectorIterations", parentRegistry);
      maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", parentRegistry);

      computationTime = new YoDouble("computationTime", parentRegistry);
      isConverged = new YoBoolean("isConverged", parentRegistry);

      maximumNumberOfIterations.set(settings.getDefaultTerminalIteration());
      solutionQualityThreshold.set(settings.getSolutionQualityThreshold());
      solutionStabilityThreshold.set(settings.getSolutionStabilityThreshold());
      solutionMinimumProgression.set(settings.getMinimumProgression());

      initialize();
   }

   public void initialize()
   {
      numberOfIterations.set(0);
      solutionQuality.set(Double.MAX_VALUE);
      isConverged.set(true);
   }

   public void update()
   {
      numberOfIterations.increment();

      // detecting rules.
      if (numberOfIterations.getIntegerValue() == maximumNumberOfIterations.getIntegerValue())
         isConverged.set(true);
      else
         isConverged.set(false);

      if (isConverged())
         initialize();
   }

   public void submitSolutionQuality(double solutionQuality)
   {
      this.solutionQuality.set(solutionQuality);
   }

   public double getComputationTime()
   {
      return computationTime.getDoubleValue();
   }

   public boolean isConverged()
   {
      return isConverged.getBooleanValue();
   }
}
