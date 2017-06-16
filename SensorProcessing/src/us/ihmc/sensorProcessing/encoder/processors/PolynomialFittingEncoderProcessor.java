package us.ihmc.sensorProcessing.encoder.processors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * See Merry, van De Molengraft, Steinbuch - 2010 - Velocity and acceleration estimation for optical incremental encoders
 * @author Twan Koolen
 *
 */
public class PolynomialFittingEncoderProcessor extends AbstractEncoderProcessor
{
   private final int nEncoderEvents;
   private final int fitOrder;
   private final int skipFactor;

   private final YoInteger[] positions;    // ordered from oldest to newest
   private final YoDouble[] timestamps;

   private final DenseMatrix64F a;
   private final DenseMatrix64F p;
   private final DenseMatrix64F b;
   private final LinearSolver<DenseMatrix64F> solver;

   private int skipIndex = 0;
   private double timespan = Double.NaN;

   public PolynomialFittingEncoderProcessor(String name, YoInteger rawPosition, YoDouble time, double distancePerTick, int nEncoderEvents,
           int fitOrder, int skipFactor, YoVariableRegistry registry)
   {
      super(name, rawPosition, time, distancePerTick, registry);

      if (fitOrder > nEncoderEvents - 1)
         throw new RuntimeException("Cannot fit a polynomial of order " + fitOrder + " through " + nEncoderEvents + " encoder events.");

      this.nEncoderEvents = nEncoderEvents;
      this.fitOrder = fitOrder;
      this.skipFactor = skipFactor;

      positions = new YoInteger[nEncoderEvents];
      timestamps = new YoDouble[nEncoderEvents];

      for (int i = 0; i < nEncoderEvents; i++)
      {
         positions[i] = new YoInteger(name + "Pos" + i, registry);
         timestamps[i] = new YoDouble(name + "Time" + i, registry);
      }

      a = new DenseMatrix64F(nEncoderEvents, fitOrder + 1);
      b = new DenseMatrix64F(a.getNumRows(), 1);
      p = new DenseMatrix64F(a.getNumCols(), 1);
      solver = LinearSolverFactory.leastSquares(a.getNumRows(), a.getNumCols());
   }
   
   /**
    * Initializes all timestamps to slightly negative but monotonically increasing values.
    * Initializes all positions to the current raw position value.
    */
   public void initialize()
   {
      int currentPosition = rawTicks.getIntegerValue();
      for (YoInteger position : positions)
      {
         position.set(currentPosition);
      }

      final double epsilonTimeInit = 1e-10;
      double initTime = time.getDoubleValue() - (nEncoderEvents) * epsilonTimeInit;
      for (YoDouble timeStamp : timestamps)
      {
         timeStamp.set(initTime);
         initTime += epsilonTimeInit;
      }
   }

   public void update()
   {
      updateYoVars();
      computePolynomialCoefficients();
      computeProcessedPosition();
      computeProcessedRate();
   }

   private void updateYoVars()
   {
      final int currentPosition = rawTicks.getIntegerValue();

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
      positions[nEncoderEvents - 1].set(rawTicks.getIntegerValue());
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

      solver.setA(a);
      solver.solve(b, p);

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

      final int rawPosition = this.rawTicks.getIntegerValue();
      if (Math.abs(ret - rawPosition) > 1)
         ret = rawPosition;

      processedTicks.set(ret);
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

      ret = limitVelocity(ret);

      processedTickRate.set(ret);
   }

   private double limitVelocity(double velocity)
   {
      double dt = time.getDoubleValue() - timestamps[nEncoderEvents - 1].getDoubleValue();
      double epsilonSameTick = 1e-14;
      if (dt > epsilonSameTick)
      {
         double maxAbsRate = 1.0 / (time.getDoubleValue() - timestamps[nEncoderEvents - 1].getDoubleValue());
         if (Math.abs(velocity) > maxAbsRate)
            velocity = maxAbsRate * Math.signum(velocity);
      }

      return velocity;
   }
}
