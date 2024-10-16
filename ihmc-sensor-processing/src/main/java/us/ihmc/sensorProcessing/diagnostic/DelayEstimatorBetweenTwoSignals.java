package us.ihmc.sensorProcessing.diagnostic;

import org.apache.commons.math3.stat.regression.SimpleRegression;
import org.ejml.data.DMatrixRMaj;

import us.ihmc.yoVariables.filters.AlphaFilterTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This class aims to estimate the delay (lag or lead) between two given signals.
 * It is achieved by finding the maximum cross-correlation between the two signals by shifting one with respect to the other.
 * For each iteration, an initial guess for the delay is used as a seed to find the local maximum for the cross-correlation.
 * However this estimated delay has a useful meaning only if there is sufficient correlation between the two signals.
 * @author Sylvain
 *
 */
public class DelayEstimatorBetweenTwoSignals implements DiagnosticUpdatable
{
   private static final int DEFAULT_MAX_ABS_LEAD = 0;
   private static final int DEFAULT_MAX_ABS_LAG = 25;
   private static final int DEFAULT_NUMBER_OF_OBSERVATIONS = 100;

   private final YoDouble referenceSignal;
   private final YoDouble delayedSignal;
   private final YoDouble estimatedDelay;

   private final YoBoolean enabled;

   private final DMatrixRMaj referenceSignalBuffer;
   private final DMatrixRMaj delayedSignalBuffer;
   private final SimpleRegression correlationCalculator = new SimpleRegression();
   private final YoDouble correlationForDelay;
   private final YoDouble maxCorrelation;
   private final YoInteger nTicksOfDelay;

   private final YoInteger maxLeadInTicks;
   private final YoInteger maxLagInTicks;
   private final YoInteger numberOfObservations;

   private final YoDouble correlationAlpha;
   private final DMatrixRMaj correlationBuffer;
   private final DMatrixRMaj filteredCorrelationBuffer;

   private int bufferPosition = 0;
   private boolean hasBufferBeenFilled = false;

   private final double dt;

   public DelayEstimatorBetweenTwoSignals(String namePrefix, double dt, YoRegistry registry)
   {
      this(namePrefix, null, null, dt, registry);
   }

   /**
    * @param referenceSignal is the signal used as ground truth to estimate the delay of {@code delayedSignal}.
    * @param delayedSignal is the signal for which the delay is estimated.
    */
   public DelayEstimatorBetweenTwoSignals(String namePrefix, YoDouble referenceSignal, YoDouble delayedSignal, double dt, YoRegistry registry)
   {
      this.dt = dt;
      this.referenceSignal = referenceSignal;
      this.delayedSignal = delayedSignal;

      nTicksOfDelay = new YoInteger(namePrefix + "_estimatedNTicksOfDelay", registry);
      estimatedDelay = new YoDouble(namePrefix + "_estimatedDelay", registry);
      correlationForDelay = new YoDouble(namePrefix + "_correlationForDelay", registry);
      maxCorrelation = new YoDouble(namePrefix + "_maxCorrelation", registry);
      enabled = new YoBoolean(namePrefix + "_enabled", registry);

      maxLeadInTicks = new YoInteger(namePrefix + "_maxPhaseLeadInTicks", registry);
      maxLeadInTicks.set(DEFAULT_MAX_ABS_LEAD);
      maxLagInTicks = new YoInteger(namePrefix + "_maxPhaseLagInTicks", registry);
      maxLagInTicks.set(DEFAULT_MAX_ABS_LAG);
      numberOfObservations = new YoInteger(namePrefix + "_nObservationsForDelayEstimation", registry);
      numberOfObservations.set(DEFAULT_NUMBER_OF_OBSERVATIONS);

      int bufferSize = numberOfObservations.getIntegerValue() + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue();
      referenceSignalBuffer = new DMatrixRMaj(bufferSize, 1);
      delayedSignalBuffer = new DMatrixRMaj(bufferSize, 1);

      correlationAlpha = new YoDouble(namePrefix + "CorrelationApha", registry);
      correlationAlpha.set(AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(0.16, dt));
      correlationBuffer = new DMatrixRMaj(1 + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue(), 1);
      filteredCorrelationBuffer = new DMatrixRMaj(1 + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue(), 1);
   }

   @Override
   public void enable()
   {
      enabled.set(true);
   }

   @Override
   public void disable()
   {
      enabled.set(false);
      reset();
   }

   public void setAlphaFilterBreakFrequency(double breakFrequency)
   {
      correlationAlpha.set(AlphaFilterTools.computeAlphaGivenBreakFrequencyProperly(breakFrequency, dt));
   }

   public void setEstimationParameters(int maxAbsoluteLeadInTicks, int maxAbsoluteLagInTicks, int numberOfObservations)
   {
      maxLeadInTicks.set(maxAbsoluteLeadInTicks);
      maxLagInTicks.set(maxAbsoluteLagInTicks);
      this.numberOfObservations.set(numberOfObservations);

      int bufferSize = this.numberOfObservations.getIntegerValue() + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue();
      referenceSignalBuffer.reshape(bufferSize, 1);
      delayedSignalBuffer.reshape(bufferSize, 1);

      correlationBuffer.reshape(1 + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue(), 1);
      filteredCorrelationBuffer.reshape(1 + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue(), 1);
   }

   public void setEstimationParameters(double maxAbsoluteLead, double maxAbsoluteLag, double observationWindow)
   {
      maxLeadInTicks.set((int) (maxAbsoluteLead / dt));
      maxLagInTicks.set((int) (maxAbsoluteLag / dt));
      this.numberOfObservations.set((int) (observationWindow / dt));

      int bufferSize = this.numberOfObservations.getIntegerValue() + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue();
      referenceSignalBuffer.reshape(bufferSize, 1);
      delayedSignalBuffer.reshape(bufferSize, 1);

      correlationBuffer.reshape(1 + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue(), 1);
      filteredCorrelationBuffer.reshape(1 + maxLeadInTicks.getIntegerValue() + maxLagInTicks.getIntegerValue(), 1);
   }

   public void reset()
   {
      bufferPosition = 0;
      hasBufferBeenFilled = false;
      firstEstimationTick = true;
   }

   @Override
   public void update()
   {
      update(referenceSignal.getDoubleValue(), delayedSignal.getDoubleValue());
   }


   public void update(double referenceSignalCurrentPosition, double delayedSignalCurrentPosition)
   {
      if (!enabled.getBooleanValue())
         return;

      referenceSignalBuffer.set(bufferPosition, 0, referenceSignalCurrentPosition);
      delayedSignalBuffer.set(bufferPosition, 0, delayedSignalCurrentPosition);

      bufferPosition++;
      if (bufferPosition >= referenceSignalBuffer.getNumRows())
      {
         bufferPosition = 0;
         hasBufferBeenFilled = true;
      }

      if (!hasBufferBeenFilled)
      {
         estimatedDelay.set(0.0);
         return;
      }

      updateCorrelationBuffer();
      findDelayFromInitialGuess();

      estimatedDelay.set(nTicksOfDelay.getIntegerValue() * dt);
   }

   private boolean firstEstimationTick = true;

   private void updateCorrelationBuffer()
   {
      double maxCorr = Double.NEGATIVE_INFINITY;

      for (int offset = -maxLagInTicks.getIntegerValue(); offset <= maxLeadInTicks.getIntegerValue(); offset++)
      {
         int index = offset + maxLagInTicks.getIntegerValue();
         correlationCalculator.clear();

         for (int i = 1; i <= numberOfObservations.getIntegerValue(); i++)
         {
            int indexX = (bufferPosition + i + offset + maxLagInTicks.getIntegerValue()) % referenceSignalBuffer.getNumRows();
            double x = referenceSignalBuffer.get(indexX, 0);
            int indexY = (bufferPosition + i + maxLagInTicks.getIntegerValue()) % delayedSignalBuffer.getNumRows();
            double y = delayedSignalBuffer.get(indexY, 0);

            correlationCalculator.addData(x, y);
         }

         double currentCorrelation = correlationCalculator.getR();
         correlationBuffer.set(index, 0, currentCorrelation);
         if (Double.isNaN(currentCorrelation))
            currentCorrelation = 0.0;
         double currentFilteredCorrelation;
         if (firstEstimationTick)
         {
            currentFilteredCorrelation = currentCorrelation;
         }
         else
         {
            double previousFilteredCorrelation = filteredCorrelationBuffer.get(index, 0);
            currentFilteredCorrelation = correlationAlpha.getDoubleValue() * previousFilteredCorrelation + (1.0 - correlationAlpha.getDoubleValue()) * currentCorrelation;
         }
         filteredCorrelationBuffer.set(index, 0, currentFilteredCorrelation);

         maxCorr = Math.max(maxCorr, currentFilteredCorrelation);
      }

      firstEstimationTick = false;
      maxCorrelation.set(maxCorr);
   }

   private void findDelayFromInitialGuess()
   {
      int initialOffsetGuess = -nTicksOfDelay.getIntegerValue();
      double initialGuessCorrelation = filteredCorrelationBuffer.get(initialOffsetGuess + maxLagInTicks.getIntegerValue());

      double maxCorrelationCoefficient = initialGuessCorrelation;
      int maxCorrelationIndex = initialOffsetGuess;

      double previousCorrelation = initialGuessCorrelation;

      int counter = 0;
      int maxBeforeBreak = 5;

      for (int offset = initialOffsetGuess + 1; offset <= maxLeadInTicks.getIntegerValue(); offset++)
      {
         double correlation = filteredCorrelationBuffer.get(offset + maxLagInTicks.getIntegerValue());

         if (correlation < previousCorrelation)
            counter++;

         if (counter == maxBeforeBreak)
            break;

         if (correlation > maxCorrelationCoefficient)
         {
            maxCorrelationCoefficient = correlation;
            maxCorrelationIndex = offset;
         }

         previousCorrelation = correlation;
      }

      previousCorrelation = initialGuessCorrelation;

      counter = 0;

      for (int offset = initialOffsetGuess; offset >= -maxLagInTicks.getIntegerValue(); offset--)
      {
         double correlation = filteredCorrelationBuffer.get(offset + maxLagInTicks.getIntegerValue());

         if (correlation < previousCorrelation)
            counter++;

         if (counter == maxBeforeBreak)
            break;

         if (correlation > maxCorrelationCoefficient)
         {
            maxCorrelationCoefficient = correlation;
            maxCorrelationIndex = offset;
         }

         previousCorrelation = correlation;
      }

      nTicksOfDelay.set(-maxCorrelationIndex);
      correlationForDelay.set(maxCorrelationCoefficient);
   }

   public boolean isEstimatingDelay()
   {
      return hasBufferBeenFilled;
   }

   /**
    * @return The delay between the two signals in seconds. If {@link #delayedSignal} has a phase lag it is positive, it is negative otherwise.
    */
   public double getEstimatedDelay()
   {
      return estimatedDelay.getDoubleValue();
   }

   /**
    * @return The best correlation value which can be used to control quality of the current estimation.
    */
   public double getCorrelationCoefficient()
   {
      return correlationForDelay.getDoubleValue();
   }
}
