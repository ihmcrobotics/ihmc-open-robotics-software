package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.filters.FilteredVelocityYoVariable;
import us.ihmc.robotics.math.filters.SimpleMovingAverageFilteredYoVariable;

public class PositionVelocity1DConsistencyChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;

   private final FilteredVelocityYoVariable referenceVelocity;

   private final SimpleMovingAverageFilteredYoVariable filteredRawVelocityToCheck;
   private final SimpleMovingAverageFilteredYoVariable filteredProcessedVelocityToCheck;
   private final SimpleMovingAverageFilteredYoVariable referenceVelocityFiltered;

   private final DelayEstimatorBetweenTwoSignals delayEstimatorProcessedPosition;
   private final DelayEstimatorBetweenTwoSignals delayEstimatorRawVelocity;
   private final DelayEstimatorBetweenTwoSignals delayEstimatorProcessedVelocity;

   private final double dt;

   public PositionVelocity1DConsistencyChecker(String namePrefix, YoDouble rawPosition, YoDouble rawVelocityToCheck, YoDouble processedPositionToCheck, YoDouble processedVelocityToCheck, double dt,
         YoVariableRegistry parentRegistry)
   {
      this.dt = dt;
      registry = new YoVariableRegistry(namePrefix + "PositionVelocity1DCheck");

      referenceVelocity = new FilteredVelocityYoVariable(namePrefix + "_referenceVelocity", "", 0.0, rawPosition, dt, registry);
      int windowSize = 10;
      referenceVelocityFiltered = new SimpleMovingAverageFilteredYoVariable(namePrefix + "_referenceVelocityFiltered", windowSize, referenceVelocity, registry);

      filteredRawVelocityToCheck = new SimpleMovingAverageFilteredYoVariable(namePrefix + "_filteredRawVelocity", windowSize, rawVelocityToCheck, registry);
      filteredProcessedVelocityToCheck = new SimpleMovingAverageFilteredYoVariable(namePrefix + "_filteredProcessedVelocity", windowSize, processedVelocityToCheck, registry);

      delayEstimatorProcessedPosition = new DelayEstimatorBetweenTwoSignals(namePrefix + "ProcessedPosition", rawPosition, processedPositionToCheck, dt, registry);
      delayEstimatorRawVelocity = new DelayEstimatorBetweenTwoSignals(namePrefix + "RawVelocity", referenceVelocityFiltered, filteredRawVelocityToCheck, dt, registry);
      delayEstimatorProcessedVelocity = new DelayEstimatorBetweenTwoSignals(namePrefix + "ProcessedVelocity", referenceVelocityFiltered, filteredProcessedVelocityToCheck, dt, registry);

      parentRegistry.addChild(registry);
   }

   @Override
   public void enable()
   {
      delayEstimatorProcessedPosition.enable();
      delayEstimatorRawVelocity.enable();
      delayEstimatorProcessedVelocity.enable();
   }

   @Override
   public void disable()
   {
      delayEstimatorProcessedPosition.disable();
      delayEstimatorRawVelocity.disable();
      delayEstimatorProcessedVelocity.disable();
   }

   public void setInputSignalsSMAWindow(double window)
   {
      int windowSize = (int) (window / dt);
      referenceVelocityFiltered.setWindowSize(windowSize);
      filteredRawVelocityToCheck.setWindowSize(windowSize);
      filteredProcessedVelocityToCheck.setWindowSize(windowSize);
   }

   public void setDelayEstimatorAlphaFilterBreakFrequency(double breakFrequency)
   {
      delayEstimatorProcessedPosition.setAlphaFilterBreakFrequency(breakFrequency);
      delayEstimatorRawVelocity.setAlphaFilterBreakFrequency(breakFrequency);
      delayEstimatorProcessedVelocity.setAlphaFilterBreakFrequency(breakFrequency);
   }

   public void setDelayEstimationParameters(double maxAbsoluteLead, double maxAbsoluteLag, double observationWindow)
   {
      delayEstimatorProcessedPosition.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
      delayEstimatorRawVelocity.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
      delayEstimatorProcessedVelocity.setEstimationParameters(maxAbsoluteLead, maxAbsoluteLag, observationWindow);
   }

   @Override
   public void update()
   {
      referenceVelocity.update();
      referenceVelocityFiltered.update();

      filteredRawVelocityToCheck.update();
      filteredProcessedVelocityToCheck.update();

      if (!referenceVelocityFiltered.getHasBufferWindowFilled())
         return;

      delayEstimatorProcessedPosition.update();
      delayEstimatorRawVelocity.update();
      delayEstimatorProcessedVelocity.update();
   }

   public boolean isEstimatingDelay()
   {
      return delayEstimatorProcessedPosition.isEstimatingDelay() && delayEstimatorRawVelocity.isEstimatingDelay() && delayEstimatorProcessedVelocity.isEstimatingDelay();
   }

   public double getEstimatedDelayForProcessedPosition()
   {
      return delayEstimatorProcessedPosition.getEstimatedDelay();
   }

   public double getConsistencyQualityForProcessedPosition()
   {
      return delayEstimatorProcessedPosition.getCorrelationCoefficient();
   }

   public double getEstimatedDelayForRawVelocity()
   {
      return delayEstimatorRawVelocity.getEstimatedDelay();
   }

   public double getConsistencyQualityForRawVelocity()
   {
      return delayEstimatorRawVelocity.getCorrelationCoefficient();
   }

   public double getEstimatedDelayForProcessedVelocity()
   {
      return delayEstimatorProcessedVelocity.getEstimatedDelay();
   }

   public double getConsistencyQualityForProcessedVelocity()
   {
      return delayEstimatorProcessedVelocity.getCorrelationCoefficient();
   }
}
