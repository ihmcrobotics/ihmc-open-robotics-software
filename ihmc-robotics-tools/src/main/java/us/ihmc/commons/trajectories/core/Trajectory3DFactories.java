package us.ihmc.commons.trajectories.core;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.commons.time.TimeIntervalBasics;
import us.ihmc.commons.time.TimeIntervalProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.function.DoubleConsumer;

import static us.ihmc.euclid.tools.EuclidHashCodeTools.toIntHashCode;

public class Trajectory3DFactories
{
   public static TimeIntervalBasics newLinkedTimeInterval(TimeIntervalProvider xInterval,
                                                          TimeIntervalProvider yInterval,
                                                          TimeIntervalProvider zInterval)
   {
      return newLinkedTimeInterval(xInterval.getTimeInterval(), yInterval.getTimeInterval(), zInterval.getTimeInterval());
   }

   private static TimeIntervalBasics newLinkedTimeInterval(TimeIntervalBasics xInterval,
                                                          TimeIntervalBasics yInterval,
                                                          TimeIntervalBasics zInterval)
   {
      return new TimeIntervalBasics()
      {
         @Override
         public void setStartTime(double startTime)
         {
            xInterval.setStartTime(startTime);
            yInterval.setStartTime(startTime);
            zInterval.setStartTime(startTime);
         }

         @Override
         public void setEndTime(double endTime)
         {
            xInterval.setEndTime(endTime);
            yInterval.setEndTime(endTime);
            zInterval.setEndTime(endTime);
         }

         @Override
         public double getStartTime()
         {
            if (!MathTools.epsilonEquals(xInterval.getStartTime(), yInterval.getStartTime(), 1e-5) ||
                !MathTools.epsilonEquals(xInterval.getStartTime(), zInterval.getStartTime(), 1e-5))
               throw new RuntimeException("Time intervals are wrong.");

            return xInterval.getStartTime();
         }

         @Override
         public double getEndTime()
         {
            if (!MathTools.epsilonEquals(xInterval.getEndTime(), yInterval.getEndTime(), 1e-5) ||
                !MathTools.epsilonEquals(xInterval.getEndTime(), zInterval.getEndTime(), 1e-5))
               throw new RuntimeException("Time intervals are wrong.");

            return xInterval.getEndTime();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof TimeIntervalBasics)
               return equals((TimeIntervalBasics) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getStartTime(), getEndTime()));
         }

         @Override
         public String toString()
         {
            return xInterval.toString();
         }
      };
   }

   public static Point3DBasics newLinkedPoint3DBasics(DoubleProvider xProvider,
                                                      DoubleConsumer xConsumer,
                                                      DoubleProvider yProvider,
                                                      DoubleConsumer yConsumer,
                                                      DoubleProvider zProvider,
                                                      DoubleConsumer zConsumer)
   {
      return new Point3DBasics()
      {
         @Override
         public void setX(double x)
         {
            xConsumer.accept(x);
         }

         @Override
         public void setY(double y)
         {
            yConsumer.accept(y);
         }

         @Override
         public void setZ(double z)
         {
            zConsumer.accept(z);
         }

         @Override
         public double getX()
         {
            return xProvider.getValue();
         }

         @Override
         public double getY()
         {
            return yProvider.getValue();
         }

         @Override
         public double getZ()
         {
            return zProvider.getValue();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof Point3DReadOnly)
               return equals((Point3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()));
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   public static Vector3DBasics newLinkedVector3DBasics(DoubleProvider xProvider,
                                                        DoubleConsumer xConsumer,
                                                        DoubleProvider yProvider,
                                                        DoubleConsumer yConsumer,
                                                        DoubleProvider zProvider,
                                                        DoubleConsumer zConsumer)
   {
      return new Vector3DBasics()
      {
         @Override
         public void setX(double x)
         {
            xConsumer.accept(x);
         }

         @Override
         public void setY(double y)
         {
            yConsumer.accept(y);
         }

         @Override
         public void setZ(double z)
         {
            zConsumer.accept(z);
         }

         @Override
         public double getX()
         {
            return xProvider.getValue();
         }

         @Override
         public double getY()
         {
            return yProvider.getValue();
         }

         @Override
         public double getZ()
         {
            return zProvider.getValue();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object == this)
               return true;
            else if (object instanceof Point3DReadOnly)
               return equals((Point3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            return toIntHashCode(toIntHashCode(getX(), getY(), getZ()));
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }
}
