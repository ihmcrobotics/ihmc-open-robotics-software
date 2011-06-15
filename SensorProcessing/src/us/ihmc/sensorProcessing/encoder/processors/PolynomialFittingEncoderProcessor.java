package us.ihmc.sensorProcessing.encoder.processors;

import com.mathworks.jama.Matrix;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

/**
 * See Merry, van De Molengraft, Steinbuch - 2010 - Velocity and acceleration estimation for optical incremental encoders
 * @author Twan Koolen
 *
 */
public class PolynomialFittingEncoderProcessor implements EncoderProcessor
{
   private double unitDistancePerCount = 1.0;
   
   private final IntYoVariable rawPosition;
   private final DoubleYoVariable time;

   private final int nEncoderEvents;
   private final int fitOrder;
   private final int skipFactor;
   private final boolean limitVelocity;

   private final IntYoVariable[] positions;    // ordered from oldest to newest
   private final DoubleYoVariable[] timestamps;

   private final DoubleYoVariable processedPosition, processedRate;

   private final Matrix a;
   private Matrix p;
   private final Matrix b;

   private boolean firstTick = true;
   private int skipIndex = 0;
   private double timespan = Double.NaN;

   public PolynomialFittingEncoderProcessor(String name, IntYoVariable rawPosition, DoubleYoVariable time, int nEncoderEvents, int fitOrder, int skipFactor,
           boolean limitVelocity, YoVariableRegistry registry)
   {
      if (fitOrder > nEncoderEvents - 1)
         throw new RuntimeException("Cannot fit a polynomial of order " + fitOrder + " through " + nEncoderEvents + " encoder events.");
      
      this.rawPosition = rawPosition;
      this.time = time;
      this.nEncoderEvents = nEncoderEvents;
      this.fitOrder = fitOrder;
      this.skipFactor = skipFactor;
      this.limitVelocity = limitVelocity;

      positions = new IntYoVariable[nEncoderEvents];
      timestamps = new DoubleYoVariable[nEncoderEvents];

      for (int i = 0; i < nEncoderEvents; i++)
      {
         positions[i] = new IntYoVariable(name + "Pos" + i, registry);
         timestamps[i] = new DoubleYoVariable(name + "Time" + i, registry);
      }

      this.processedPosition = new DoubleYoVariable(name + "procPos", registry);
      this.processedRate = new DoubleYoVariable(name + "procRate", registry);

      a = new Matrix(nEncoderEvents, fitOrder + 1);
      b = new Matrix(nEncoderEvents, 1);
   }

   public void update()
   {
      updateYoVars();
      computePolynomialCoefficients();
      computeProcessedPosition();
      computeProcessedRate();
   }

   public double getQ()
   {
      return processedPosition.getDoubleValue() * unitDistancePerCount;
   }

   public double getQd()
   {
      return processedRate.getDoubleValue() * unitDistancePerCount;
   }

   private void updateYoVars()
   {
      final int currentPosition = rawPosition.getIntegerValue();

      if (firstTick)
      {
         initializeFirstTick();

         firstTick = false;
      }

      int lastPosition = positions[nEncoderEvents - 1].getIntegerValue();
      if (currentPosition != lastPosition)
      {
         if (skipIndex == skipFactor)
         {
            shiftAndUpdatePositions();
            shiftAndUpdateTimestamps();
            skipIndex = 0;
         }
         else
         {
            updateLatestPosition();
            updateLatestTimestamp();
            skipIndex++;
         }
      }
   }

   /**
    * Initializes all timestamps to slightly negative but monotonically increasing values.
    * Initializes all positions to the current raw position value.
    */
   private void initializeFirstTick()
   {
      int currentPosition = rawPosition.getIntegerValue();
      for (IntYoVariable position : positions)
      {
         position.set(currentPosition);
      }

      final double epsilonTimeInit = 1e-10;
      double initTime = time.getDoubleValue() - (nEncoderEvents) * epsilonTimeInit;
      for (DoubleYoVariable timeStamp : timestamps)
      {
         timeStamp.set(initTime);
         initTime += epsilonTimeInit;
      }
   }

   private void shiftAndUpdatePositions()
   {
      for (int i = 0; i < nEncoderEvents - 1; i++)
      {
         positions[i].set(positions[i + 1].getIntegerValue());
      }

      updateLatestPosition();
   }

   private void shiftAndUpdateTimestamps()
   {
      for (int i = 0; i < nEncoderEvents - 1; i++)
      {
         timestamps[i].set(timestamps[i + 1].getDoubleValue());
      }

      updateLatestTimestamp();
   }


   private void updateLatestPosition()
   {
      positions[nEncoderEvents - 1].set(rawPosition.getIntegerValue());
   }

   private void updateLatestTimestamp()
   {
      timestamps[nEncoderEvents - 1].set(time.getDoubleValue());
   }

   private void computePolynomialCoefficients()
   {
      timespan = timestamps[nEncoderEvents - 1].getDoubleValue() - timestamps[0].getDoubleValue();

      for (int i = 0; i < nEncoderEvents; i++)
      {
         double timestamp = timestamps[i].getDoubleValue();
         timestamp /= timespan;

         double entry = 1.0;
         for (int j = fitOrder; j >= 0; j--)
         {
            a.set(i, j, entry);
            if (j > 0)
               entry *= timestamp;
         }
      }

      for (int i = 0; i < nEncoderEvents; i++)
      {
         b.set(i, 0, positions[i].getIntegerValue());
      }

      p = a.solve(b);
      // TODO: can probably make this more efficient by using the structure of the matrix a...
   }

   private void computeProcessedPosition()
   {
      double x = time.getDoubleValue() / timespan;

      double x_n = 1.0;
      double ret = 0.0;

      for (int i = fitOrder; i >= 0; i--)
      {
         double coefficient = p.get(i, 0);
         ret = ret + coefficient * x_n;
         x_n = x_n * x;
      }

      final int rawPosition = this.rawPosition.getIntegerValue();
      if (Math.abs(ret - rawPosition) > 1)
         ret = rawPosition;

      processedPosition.set(ret);
   }

   private void computeProcessedRate()
   {
      double x = time.getDoubleValue() / timespan;

      double x_n = 1.0;
      double ret = 0.0;

      for (int i = fitOrder - 1; i >= 0; i--)
      {
         final int exponent = fitOrder - i;
         double coefficient = p.get(i, 0) * exponent / timespan;
         ret = ret + coefficient * x_n;
         x_n = x_n * x;
      }
      
      if (limitVelocity)
      {
         double dt = time.getDoubleValue() - timestamps[nEncoderEvents - 1].getDoubleValue();
         double epsilonSameTick = 1e-14;
         if (dt > epsilonSameTick)
         {
            double maxAbsRate = 1.0 / (time.getDoubleValue() - timestamps[nEncoderEvents - 1].getDoubleValue());
            if (Math.abs(ret) > maxAbsRate)
               ret = maxAbsRate * Math.signum(ret);
         }
      }

      processedRate.set(ret);
   }
   
   public void setUnitDistancePerCount(double unitDistancePerCount)
   {
      this.unitDistancePerCount = unitDistancePerCount;
   }

}
