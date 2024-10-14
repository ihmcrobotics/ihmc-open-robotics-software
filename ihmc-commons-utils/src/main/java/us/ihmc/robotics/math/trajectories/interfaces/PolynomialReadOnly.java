package us.ihmc.robotics.math.trajectories.interfaces;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.commons.MathTools;
import us.ihmc.robotics.math.trajectories.core.PolynomialMath;
import us.ihmc.robotics.time.TimeIntervalProvider;

public interface PolynomialReadOnly extends TimeIntervalProvider, DoubleTrajectoryGenerator
{
   int getMaximumNumberOfCoefficients();

   int getNumberOfCoefficients();

   default int getOrder()
   {
      return getNumberOfCoefficients() - 1;
   }

   double getCurrentTime();

   double getCoefficient(int idx);

   double getDerivativeCoefficient(int idx);

   double getDoubleDerivativeCoefficient(int idx);

   double[] getCoefficients();

   DMatrixRMaj getCoefficientsVector();

   default double getFinalTime()
   {
      return getTimeInterval().getEndTime();
   }

   default double getInitialTime()
   {
      return getTimeInterval().getStartTime();
   }

   default double getDuration()
   {
      return getTimeInterval().getDuration();
   }

   default boolean timeIntervalContains(double timeToCheck, double EPSILON)
   {
      return getTimeInterval().epsilonContains(timeToCheck, EPSILON);
   }

   default boolean timeIntervalContains(double timeToCheck)
   {
      return getTimeInterval().intervalContains(timeToCheck);
   }

   default PolynomialBasics times(PolynomialReadOnly other)
   {
      return PolynomialMath.times(this, other);
   }

   default PolynomialBasics times(double value)
   {
      return PolynomialMath.times(this, value);
   }

   default PolynomialBasics plus(PolynomialReadOnly other)
   {
      return PolynomialMath.plus(this, other);
   }

   @Override
   default boolean isDone()
   {
      return getCurrentTime() >= getTimeInterval().getEndTime();
   }

   default boolean epsilonEquals(PolynomialReadOnly other, double epsilon)
   {
      int minSize = Math.min(getNumberOfCoefficients(), other.getNumberOfCoefficients());
      int maxSize = Math.max(getNumberOfCoefficients(), other.getNumberOfCoefficients());

      for (int i = 0; i < minSize; i++)
      {
         if (!MathTools.epsilonEquals(getCoefficient(i), other.getCoefficient(i), epsilon))
            return false;
      }
      for (int i = minSize; i < maxSize; i++)
      {
         if (maxSize == getNumberOfCoefficients())
         {
            if (!MathTools.epsilonEquals(getCoefficient(i), 0.0, epsilon))
               return false;
         }
         else
         {
            if (!MathTools.epsilonEquals(other.getCoefficient(i), 0.0, epsilon))
               return false;
         }
      }

      return true;
   }

   default boolean equalsZero(double epsilon)
   {
      return Math.abs(getCoefficient(0)) < epsilon;
   }
}
