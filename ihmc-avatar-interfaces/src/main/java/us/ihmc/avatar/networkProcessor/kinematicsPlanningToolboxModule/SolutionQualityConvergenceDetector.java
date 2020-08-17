package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import us.ihmc.commons.Conversions;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class SolutionQualityConvergenceDetector
{
   private final String name;

   private final YoBoolean useConvergenceDetecting;

   private final YoDouble solutionQuality;
   private final YoDouble solutionQualityLast;
   private final YoDouble solutionQualityBeforeLast;

   private final YoDouble solutionQualityThreshold;
   private final YoDouble solutionStabilityThreshold;
   private final YoDouble solutionMinimumProgression;

   private final YoInteger numberOfIterations;
   private final YoInteger maximumNumberOfIterations;

   private final YoDouble computationTime;

   private final YoBoolean isSolved;
   private final YoBoolean isGoodSolution;
   private final YoBoolean isStucked;

   public SolutionQualityConvergenceDetector(SolutionQualityConvergenceSettings settings, YoRegistry parentRegistry)
   {
      name = this.getClass().getSimpleName();

      useConvergenceDetecting = new YoBoolean("useConvergenceDetecting", parentRegistry);

      solutionQuality = new YoDouble(name + "solutionQuality", parentRegistry);
      solutionQualityLast = new YoDouble(name + "solutionQualityLast", parentRegistry);
      solutionQualityBeforeLast = new YoDouble(name + "solutionQualityBeforeLast", parentRegistry);

      solutionQualityThreshold = new YoDouble("solutionQualityThreshold", parentRegistry);
      solutionStabilityThreshold = new YoDouble("solutionStabilityThreshold", parentRegistry);
      solutionMinimumProgression = new YoDouble("solutionProgressionThreshold", parentRegistry);

      numberOfIterations = new YoInteger(name + "numberOfIterations", parentRegistry);
      maximumNumberOfIterations = new YoInteger("maximumNumberOfIterations", parentRegistry);

      computationTime = new YoDouble("computationTime", parentRegistry);

      isSolved = new YoBoolean("isSolved", parentRegistry);
      isGoodSolution = new YoBoolean("isGoodSolution", parentRegistry);
      isStucked = new YoBoolean("isStucked", parentRegistry);

      useConvergenceDetecting.set(settings.useConvergenceDetecting());

      maximumNumberOfIterations.set(settings.getDefaultTerminalIteration());
      solutionQualityThreshold.set(settings.getSolutionQualityThreshold());
      solutionStabilityThreshold.set(settings.getSolutionStabilityThreshold());
      solutionMinimumProgression.set(settings.getMinimumProgression());

      initialize();
   }

   private long startTime;

   public void initialize()
   {
      startTime = System.nanoTime();

      numberOfIterations.set(0);
      solutionQuality.set(Double.MAX_VALUE);
      solutionQualityLast.setToNaN();
      solutionQualityBeforeLast.setToNaN();
      isSolved.set(false);
   }

   public void update()
   {
      numberOfIterations.increment();

      double deltaSolutionQualityLast = Math.abs(solutionQuality.getDoubleValue() - solutionQualityLast.getDoubleValue());
      double deltaSolutionQualityBeforeLast = Math.abs(solutionQuality.getDoubleValue() - solutionQualityBeforeLast.getDoubleValue());

      boolean isSolutionStable = deltaSolutionQualityLast < solutionStabilityThreshold.getDoubleValue();
      boolean isSolutionQualityHigh = solutionQuality.getDoubleValue() < solutionQualityThreshold.getDoubleValue();

      isGoodSolution.set(isSolutionStable && isSolutionQualityHigh);

      if (useConvergenceDetecting.getBooleanValue())
      {
         boolean isSolverStuck = false;

         if (!isSolutionQualityHigh)
         {
            boolean stuckLast = (deltaSolutionQualityLast / solutionQuality.getDoubleValue()) < solutionMinimumProgression.getDoubleValue();
            boolean stuckBeforeLast = (deltaSolutionQualityBeforeLast / solutionQuality.getDoubleValue()) < solutionMinimumProgression.getDoubleValue();

            isSolverStuck = stuckLast || stuckBeforeLast;
         }
         else
            isSolverStuck = false;

         isStucked.set(isSolverStuck);
         solutionQualityBeforeLast.set(solutionQualityLast.getDoubleValue());
         solutionQualityLast.set(solutionQuality.getDoubleValue());

         if (isStucked.getBooleanValue())
            isSolved.set(true);
      }

      if (numberOfIterations.getIntegerValue() == maximumNumberOfIterations.getIntegerValue() || isGoodSolution.getBooleanValue())
         isSolved.set(true);
      else
         isSolved.set(false);

      computationTime.set(Conversions.nanosecondsToSeconds(System.nanoTime() - startTime));

   }

   public void submitSolutionQuality(double solutionQuality)
   {
      this.solutionQuality.set(solutionQuality);
   }

   public double getComputationTime()
   {
      return computationTime.getDoubleValue();
   }

   public boolean isValid()
   {
      return isGoodSolution.getBooleanValue();
   }

   public boolean isSolved()
   {
      return isSolved.getBooleanValue();
   }
   
   public int getNumberOfIteration()
   {
      return numberOfIterations.getIntegerValue();
   }
}
