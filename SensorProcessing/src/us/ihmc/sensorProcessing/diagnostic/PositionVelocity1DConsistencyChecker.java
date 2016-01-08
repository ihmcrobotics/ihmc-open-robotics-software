package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;

public class PositionVelocity1DConsistencyChecker
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

      delayEstimator = new DelayEstimatorBetweenTwoSignals(namePrefix, localVelocityFiltered, filteredVelocityToCheck, dt, registry);

      parentRegistry.addChild(registry);
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

   public void update()
   {
      localVelocityFromFD.update();
      localVelocityFiltered.update();

      filteredVelocityToCheck.update();

      if (!localVelocityFiltered.getHasBufferWindowFilled())
         return;

      delayEstimator.update();
   }
}
