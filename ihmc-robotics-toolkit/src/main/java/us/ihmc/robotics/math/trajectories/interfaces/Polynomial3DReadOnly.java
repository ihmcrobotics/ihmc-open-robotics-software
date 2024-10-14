package us.ihmc.robotics.math.trajectories.interfaces;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.time.TimeIntervalProvider;

public interface Polynomial3DReadOnly extends PositionTrajectoryGenerator, TimeIntervalProvider
{
   PolynomialReadOnly getAxis(int ordinal);

   default PolynomialReadOnly getAxis(Axis3D axis)
   {
      return getAxis(axis.ordinal());
   }

   Tuple3DReadOnly[] getCoefficients();

   default void compute(double t)
   {
      for (int index = 0; index < 3; index++)
         getAxis(index).compute(t);
   }

   default boolean isDone()
   {
      for (int i = 0; i < 3; i++)
      {
         if (!getAxis(i).isDone())
            return false;
      }

      return true;
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

   default int getNumberOfCoefficients()
   {
      if (getNumberOfCoefficients(0) == getNumberOfCoefficients(1) && getNumberOfCoefficients(0) == getNumberOfCoefficients(2))
         return getNumberOfCoefficients(0);
      else
         return -1;
   }

   default int getNumberOfCoefficients(Axis3D dir)
   {
      return getAxis(dir.ordinal()).getNumberOfCoefficients();
   }

   default int getNumberOfCoefficients(int index)
   {
      return getAxis(index).getNumberOfCoefficients();
   }

   default int getMaximumNumberOfCoefficients()
   {
      if (getMaximumNumberOfCoefficients(0) == getMaximumNumberOfCoefficients(1) && getMaximumNumberOfCoefficients(0) == getMaximumNumberOfCoefficients(2))
         return getMaximumNumberOfCoefficients(0);
      else
         throw new RuntimeException("The maximum number of coefficients are not the same for all three axes.");
   }

   default int getMaximumNumberOfCoefficients(Axis3D dir)
   {
      return getAxis(dir.ordinal()).getNumberOfCoefficients();
   }

   default int getMaximumNumberOfCoefficients(int index)
   {
      return getAxis(index).getMaximumNumberOfCoefficients();
   }

   default void getCoefficients(int idx, DMatrixRMaj coefficientsToPack)
   {
      getCoefficients(idx).get(coefficientsToPack);
   }

   default Tuple3DReadOnly getCoefficients(int i)
   {
      return getCoefficients()[i];
   }
}
