package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;

public class PositionVelocity1DConsistencyChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final FilteredVelocityYoVariable localVelocityFromFD;

   private final SimpleMovingAverageFilteredYoVariable filteredVelocityToCheck;
   private final SimpleMovingAverageFilteredYoVariable localVelocityFiltered;

   private final DelayEstimatorBetweenTwoSignals delayEstimator;

   private final double dt;

   public PositionVelocity1DConsistencyChecker(String namePrefix, DoubleYoVariable position, DoubleYoVariable velocityToCheck, double dt,
         YoVariableRegistry parentRegistry)
   {
      this.dt = dt;
      registry = new YoVariableRegistry(namePrefix + "PositionVelocity1DCheck");

      localVelocityFromFD = new FilteredVelocityYoVariable(namePrefix + "_referenceFD", "", 0.0, position, dt, registry);
      int windowSize = 10;
      localVelocityFiltered = new SimpleMovingAverageFilteredYoVariable(namePrefix + "_referenceFiltered", windowSize, localVelocityFromFD, registry);
      filteredVelocityToCheck = new SimpleMovingAverageFilteredYoVariable(namePrefix + "_filtered", windowSize, velocityToCheck, registry);

      delayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix + "PositionVelocity", localVelocityFiltered, filteredVelocityToCheck, dt, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void enable()
   {
      delayEstimator.enable();
   }

   @Override
   public void disable()
   {
      delayEstimator.disable();
   }

   public void setInputSignalsSMAWindow(double window)
   {
      localVelocityFiltered.setWindowSize((int) (window / dt));
      filteredVelocityToCheck.setWindowSize((int) (window / dt));
   }

   public void setDelayEstimatorAlphaFilterBreakFrequency(double breakFrequency)
   {
      delayEstimator.setAlphaFilterBreakFrequency(breakFrequency);
   }

   public void setDelayEstimationParameters(double maxAbsoluteLead, double maxAbsoluteLag, double observationWindow)
   {
      delayEstimator.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
   }

   @Override
   public void update()
   {
      localVelocityFromFD.update();
      localVelocityFiltered.update();

      filteredVelocityToCheck.update();

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      delayEstimator.update();
   }

   public boolean isEstimatingDelay()
   {
      return delayEstimator.isEstimatingDelay();
   }

   public double getEstimatedDelay()
   {
      return delayEstimator.getEstimatedDelay();
   }

   public double getConsistencyQuality()
   {
      return delayEstimator.getCorrelationCoefficient();
   }
}
