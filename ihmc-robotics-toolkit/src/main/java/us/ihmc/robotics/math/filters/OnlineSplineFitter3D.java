package us.ihmc.robotics.math.filters;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.lists.RingBuffer;
import us.ihmc.robotics.math.filters.OnlineSplineFitter1D.DataPoint1DReadOnly;
import us.ihmc.robotics.math.filters.OnlineSplineFitter1D.SplineFitter1D;
import us.ihmc.yoVariables.providers.DoubleProvider;

import java.util.ArrayList;
import java.util.List;

public class OnlineSplineFitter3D
{
   public static final double DEFAULT_WEIGHT = 1.0;
   private double windowTimeMax;

   private boolean isSplineInitialized = false;

   private final RingBuffer<DataPoint3D> buffer;
   private final SplineFitter3D splineFitter = new SplineFitter3D();
   private WeightCalculator weightCalculator = null;

   public OnlineSplineFitter3D(int polynomialOrder, int windowSizeMax, double windowTimeMax)
   {
      setWindowTimeMax(windowTimeMax);
      setPolynomialOrder(polynomialOrder);
      buffer = new RingBuffer<>(windowSizeMax, DataPoint3D::new, DataPoint3D::set);
   }

   public void reset()
   {
      buffer.reset();
      splineFitter.clear();
      isSplineInitialized = false;
   }

   public void setWeightCalculator(WeightCalculator weightCalculator)
   {
      this.weightCalculator = weightCalculator;
   }

   public void setPolynomialOrder(int polynomialOrder)
   {
      splineFitter.setOrder(polynomialOrder);
   }

   public void setWindowSizeMax(int windowSizeMax)
   {
      buffer.changeCapacity(windowSizeMax);
   }

   public void setWindowTimeMax(double windowTimeMax)
   {
      this.windowTimeMax = windowTimeMax;
   }

   public void recordNewPoint(double time, Tuple3DReadOnly value)
   {
      recordNewPoint(time, value.getX(), value.getY(), value.getZ());
   }

   public void recordNewPoint(double time, double x, double y, double z)
   {
      DataPoint3D newPoint = buffer.add();
      newPoint.setTime(time);
      newPoint.getValue().set(x, y, z);

      DataPoint3D newestPoint = buffer.getLast();
      DataPoint3D oldestPoint = buffer.getFirst();

      if (!buffer.isBufferFull() && newestPoint.getTime() - oldestPoint.getTime() < windowTimeMax)
         return;
      if (buffer.size() < splineFitter.getNumberOfCoefficients())
         return;

      double oldestAcceptableTime = newestPoint.getTime() - windowTimeMax;
      splineFitter.clear();
      splineFitter.addPoint(newestPoint);

      for (int i = 1; i < buffer.size(); i++)
      {
         DataPoint3D point = buffer.getFromLast(i);
         if (weightCalculator != null)
         {
            double weight = weightCalculator.calculateWeight(point, newestPoint.getTime(), oldestPoint.getTime(), i, buffer.size());
            if (weight < 0.0 || Double.isNaN(weight))
               point.setWeight(DEFAULT_WEIGHT);
            else
               point.setWeight(weight);
         }
         else
         {
            point.setWeight(DEFAULT_WEIGHT);
         }

         if (point.getWeight() == 0.0)
            continue;

         splineFitter.addPoint(point);

         if (i >= splineFitter.getNumberOfCoefficients() && point.getTime() < oldestAcceptableTime)
            break;
      }

      splineFitter.fit();
      isSplineInitialized = true;
   }

   public int getNumberOfPoints()
   {
      return buffer.size();
   }

   public boolean isSplineInitialized()
   {
      return isSplineInitialized;
   }

   public double getNewestPointTime()
   {
      return buffer.getLast().getTime();
   }

   public Tuple3DReadOnly evaluateValueAt(double time)
   {
      if (isSplineInitialized)
         return splineFitter.evaluateValueAt(time);
      else if (buffer.isEmpty())
         return EuclidCoreTools.origin3D;
      else
         return buffer.getLast().getValue();
   }

   public Tuple3DReadOnly evaluateRateAt(double time)
   {
      if (isSplineInitialized)
         return splineFitter.evaluateRateAt(time);
      else
         return EuclidCoreTools.zeroVector3D;
   }

   public Tuple3DReadOnly evaluateAccelerationAt(double time)
   {
      if (isSplineInitialized)
         return splineFitter.evaluateAccelerationAt(time);
      else
         return EuclidCoreTools.zeroVector3D;
   }

   public static class SplineFitter3D
   {
      private final List<DataPoint3D> points = new ArrayList<>();
      private final SplineFitter1D xFitter = new SplineFitter1D();
      private final SplineFitter1D yFitter = new SplineFitter1D();
      private final SplineFitter1D zFitter = new SplineFitter1D();
      private final SplineFitter1D[] fitters = new SplineFitter1D[] {xFitter, yFitter, zFitter};

      public SplineFitter3D()
      {
      }

      public void setOrder(int order)
      {
         for (SplineFitter1D fitter : fitters)
            fitter.setOrder(order);
      }

      public int getOrder()
      {
         return xFitter.getOrder();
      }

      public int getNumberOfCoefficients()
      {
         return xFitter.getNumberOfCoefficients();
      }

      public void setRegularizationWeight(double regularizationWeight)
      {
         for (SplineFitter1D fitter : fitters)
            fitter.setRegularizationWeight(regularizationWeight);
      }

      public void clear()
      {
         for (SplineFitter1D fitter : fitters)
            fitter.clear();
      }

      public void addPoint(DataPoint3D point)
      {
         points.add(point);
         xFitter.addPoint(point.x());
         yFitter.addPoint(point.y());
         zFitter.addPoint(point.z());
      }

      public void addPoints(DataPoint3D[] points)
      {
         for (DataPoint3D point : points)
            addPoint(point);
      }

      public void addPoints(List<DataPoint3D> points)
      {
         for (int i = 0; i < points.size(); i++)
         {
            addPoint(points.get(i));
         }
      }

      public List<DataPoint3D> getPoints()
      {
         return points;
      }

      public void fit()
      {
         for (SplineFitter1D fitter : fitters)
            fitter.fit();
      }

      private final Point3D valueAt = new Point3D();

      public Point3D evaluateValueAt(double time)
      {
         for (int i = 0; i < fitters.length; i++)
            valueAt.setElement(i, fitters[i].evaluateValueAt(time));

         return valueAt;
      }

      private final Vector3D rateAt = new Vector3D();

      public Tuple3DReadOnly evaluateRateAt(double time)
      {
         for (int i = 0; i < fitters.length; i++)
            rateAt.setElement(i, fitters[i].evaluateRateAt(time));

         return rateAt;
      }

      private final Vector3D accelerationAt = new Vector3D();

      public Tuple3DReadOnly evaluateAccelerationAt(double time)
      {
         for (int i = 0; i < fitters.length; i++)
            accelerationAt.setElement(i, fitters[i].evaluateAccelerationAt(time));

         return accelerationAt;
      }

      public double[] getXCoefficients()
      {
         return xFitter.getCoefficients();
      }

      public double[] getYCoefficients()
      {
         return yFitter.getCoefficients();
      }

      public double[] getZCoefficients()
      {
         return zFitter.getCoefficients();
      }
   }

   public static class DataPoint3D
   {
      private double time;
      private final Point3D value = new Point3D();
      private double weight = DEFAULT_WEIGHT;

      public DataPoint3D()
      {
      }

      public DataPoint3D(DataPoint3D other)
      {
         set(other);
      }

      public DataPoint3D(double time, Tuple3DReadOnly value, double weight)
      {
         set(time, value, weight);
      }

      public void set(DataPoint3D other)
      {
         set(other.time, other.value, other.weight);
      }

      public void set(double time, Tuple3DReadOnly value, double weight)
      {
         this.time = time;
         this.value.set(value);
         this.weight = weight;
      }

      public void setTime(double time)
      {
         this.time = time;
      }

      public void setValue(Tuple3DReadOnly value)
      {
         this.value.set(value);
      }

      public void setWeight(double weight)
      {
         this.weight = weight;
      }

      public double getTime()
      {
         return time;
      }

      public Tuple3DBasics getValue()
      {
         return value;
      }

      public double getWeight()
      {
         return weight;
      }

      private DataPoint1DReadOnly xLinked;

      public DataPoint1DReadOnly x()
      {
         if (xLinked == null)
            xLinked = DataPoint1DReadOnly.link(this::getTime, value::getX, this::getWeight);
         return xLinked;
      }

      private DataPoint1DReadOnly yLinked;

      public DataPoint1DReadOnly y()
      {
         if (yLinked == null)
            yLinked = DataPoint1DReadOnly.link(this::getTime, value::getY, this::getWeight);
         return yLinked;
      }

      private DataPoint1DReadOnly zLinked;

      public DataPoint1DReadOnly z()
      {
         if (zLinked == null)
            zLinked = DataPoint1DReadOnly.link(this::getTime, value::getZ, this::getWeight);
         return zLinked;
      }

      @Override
      public String toString()
      {
         return getClass().getSimpleName() + ": time = " + time + ", value = " + value + ", weight = " + weight;
      }
   }

   public interface WeightCalculator
   {
      /**
       * Calculate the weight of a point based on its time and the time of the newest and oldest points in the buffer.
       *
       * @param point3D        the point to calculate the weight for. Not modified.
       * @param newestTime     the time of the newest point in the buffer.
       * @param oldestTime     the time of the oldest point in the buffer.
       * @param index          the index of the point in the buffer, with 0 being the newest point and buffer.size() - 1 being the oldest point.
       * @param numberOfPoints the number of points in the buffer.
       * @return the weight of the point. Return a negative value or NaN to use the default weight value.
       */
      double calculateWeight(DataPoint3D point3D, double newestTime, double oldestTime, int index, int numberOfPoints);
   }

   public static WeightCalculator createLinearWeightCalculator()
   {
      return (point3D, newestTime, oldestTime, index, numberOfPoints) -> 1.0 - (point3D.getTime() - oldestTime) / (newestTime - oldestTime);
   }

   public static WeightCalculator createExponentialWeightCalculator(double decayRate)
   {
      return (point3D, newestTime, oldestTime, index, numberOfPoints) -> Math.exp(-decayRate * (point3D.getTime() - oldestTime) / (newestTime - oldestTime));
   }

   public static WeightCalculator createExponentialWeightCalculator(DoubleProvider decayRate)
   {
      return (point3D, newestTime, oldestTime, index, numberOfPoints) -> Math.exp(
            -decayRate.getValue() * (point3D.getTime() - oldestTime) / (newestTime - oldestTime));
   }
}