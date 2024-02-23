package us.ihmc.robotics.math.filters;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.robotics.lists.RingBuffer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

public class OnlineSplineFitter1D
{
   private final double windowTimeMax;
   private boolean isSplineInitialized = false;

   private final RingBuffer<DataPoint1D> buffer;
   private final SplineFitter1D splineFitter = new SplineFitter1D();

   public OnlineSplineFitter1D(int polynomialOrder, int windowSizeMax, double windowTimeMax)
   {
      this.windowTimeMax = windowTimeMax;
      splineFitter.setOrder(polynomialOrder);
      //      splineFitter.enableRateRegularization(true);
      buffer = new RingBuffer<>(windowSizeMax, DataPoint1D::new, DataPoint1D::set);
   }

   public void recordNewPoint(double time, double value)
   {
      DataPoint1D newPoint = buffer.add();
      newPoint.setTime(time);
      newPoint.setValue(value);

      DataPoint1D newestPoint = buffer.getLast();
      DataPoint1D oldestPoint = buffer.getFirst();

      if (!buffer.isBufferFull() && newestPoint.getTime() - oldestPoint.getTime() < windowTimeMax)
         return;
      if (buffer.size() < splineFitter.getNumberOfCoefficients())
         return;

      double oldestAcceptableTime = newestPoint.getTime() - windowTimeMax;
      splineFitter.clear();
      splineFitter.addPoint(newestPoint);

      for (int i = 1; i < buffer.size(); i++)
      {
         DataPoint1D point = buffer.getFromLast(i);

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

   public double getOldestPointTime()
   {
      return buffer.getFirst().getTime();
   }

   public double evaluateValueAt(double time)
   {
      if (isSplineInitialized)
         return splineFitter.evaluateValueAt(time);
      else if (buffer.isEmpty())
         return 0.0;
      else
         return buffer.getLast().getValue();
   }

   public double evaluateRateAt(double time)
   {
      if (isSplineInitialized)
         return splineFitter.evaluateRateAt(time);
      else
         return 0.0;
   }

   public double evaluateAccelerationAt(double time)
   {
      if (isSplineInitialized)
         return splineFitter.evaluateAccelerationAt(time);
      else
         return 0.0;
   }

   public static class SplineFitter1D
   {
      private final List<DataPoint1DReadOnly> points = new ArrayList<>();
      private double[] coefficients;

      private final DMatrixRMaj H = new DMatrixRMaj(10, 10);
      private final DMatrixRMaj g = new DMatrixRMaj(10, 1);
      private final DMatrixRMaj x = new DMatrixRMaj(10, 1);

      private final DMatrixRMaj jacobian = new DMatrixRMaj(10, 10);
      private final DMatrixRMaj weight = new DMatrixRMaj(10, 10);
      private final DMatrixRMaj objective = new DMatrixRMaj(10, 1);

      private final DMatrixRMaj jtW = new DMatrixRMaj(10, 10);

      private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.symmPosDef(10);

      private double regularizationWeight = 1.0e-8;

      private final DataPoint1D min = new DataPoint1D();
      private final DataPoint1D max = new DataPoint1D();

      public SplineFitter1D()
      {
      }

      public void setOrder(int order)
      {
         int nCoeffs = order + 1;
         coefficients = new double[nCoeffs];
         H.reshape(nCoeffs, nCoeffs);
         g.reshape(nCoeffs, 1);
         x.reshape(nCoeffs, 1);
      }

      public int getOrder()
      {
         return coefficients.length - 1;
      }

      public int getNumberOfCoefficients()
      {
         return coefficients.length;
      }

      public void setRegularizationWeight(double regularizationWeight)
      {
         this.regularizationWeight = regularizationWeight;
      }

      public void clear()
      {
         points.clear();
      }

      public void addPoint(DataPoint1DReadOnly point)
      {
         points.add(point);
      }

      public void addPoints(DataPoint1DReadOnly[] points)
      {
         for (DataPoint1DReadOnly point : points)
            addPoint(point);
      }

      public void addPoints(List<? extends DataPoint1DReadOnly> points)
      {
         for (int i = 0; i < points.size(); i++)
         {
            addPoint(points.get(i));
         }
      }

      public List<DataPoint1DReadOnly> getPoints()
      {
         return points;
      }

      public void fit()
      {
         min.setTime(Double.POSITIVE_INFINITY);
         min.setValue(Double.POSITIVE_INFINITY);
         max.setTime(Double.NEGATIVE_INFINITY);
         max.setValue(Double.NEGATIVE_INFINITY);

         for (int i = 0; i < points.size(); i++)
         {
            DataPoint1DReadOnly point = points.get(i);
            min.setTime(Math.min(point.getTime(), min.getTime()));
            min.setValue(Math.min(point.getValue(), min.getValue()));
            max.setTime(Math.max(point.getTime(), max.getTime()));
            max.setValue(Math.max(point.getValue(), max.getValue()));
         }
         int nCoeffs = coefficients.length;
         jacobian.reshape(points.size(), nCoeffs);
         weight.reshape(points.size(), points.size());
         objective.reshape(points.size(), 1);
         jacobian.zero();
         weight.zero();
         objective.zero();

         for (int i = 0; i < points.size(); i++)
         {
            setValueConstraint(i, points.get(i));
         }

         jtW.reshape(nCoeffs, points.size());
         CommonOps_DDRM.multTransA(jacobian, weight, jtW);

         CommonOps_DDRM.mult(jtW, jacobian, H);
         CommonOps_DDRM.mult(jtW, objective, g);

         for (int i = 0; i < H.getNumCols(); i++)
         {
            H.add(i, i, regularizationWeight);
         }

         solver.setA(H);
         solver.solve(g, x);

         for (int i = 0; i < coefficients.length; i++)
         {
            coefficients[i] = x.get(i);
         }
      }

      public double evaluateValueAt(double time)
      {
         double x_n = 1.0;
         double value = 0.0;
         time = normalizeTime(time);

         for (int i = 0; i < coefficients.length; i++)
         {
            value += coefficients[i] * x_n;
            x_n *= time;
         }

         return value;
      }

      public double evaluateRateAt(double time)
      {
         double x_n = 1.0;
         double rate = 0.0;
         time = normalizeTime(time);

         for (int i = 1; i < coefficients.length; i++)
         {
            rate += i * coefficients[i] * x_n;
            x_n *= time;
         }

         return rate / timeScale();
      }

      public double evaluateAccelerationAt(double time)
      {
         double x_n = 1.0;
         double acceleration = 0.0;
         time = normalizeTime(time);

         for (int i = 2; i < coefficients.length; i++)
         {
            acceleration += (i - 1.0) * i * coefficients[i] * x_n;
            x_n *= time;
         }

         return acceleration / MathTools.square(timeScale());
      }

      public double[] getCoefficients()
      {
         return coefficients;
      }

      private void setValueConstraint(int row, DataPoint1DReadOnly point)
      {
         double x_n = 1.0;
         double time = normalizeTime(point.getTime());

         for (int column = 0; column < coefficients.length; column++)
         {
            jacobian.set(row, column, x_n);
            x_n *= time;
         }

         weight.set(row, row, point.getWeight());
         objective.set(row, point.getValue());
      }

      private double normalizeTime(double time)
      {
         return (time - min.getTime()) / timeScale();
      }

      private double timeScale()
      {
         return max.getTime() - min.getTime();
      }
   }

   public static class DataPoint1D implements DataPoint1DReadOnly
   {
      private double time;
      private double value;
      private double weight = 1.0;

      public DataPoint1D()
      {
      }

      public DataPoint1D(DataPoint1D other)
      {
         set(other);
      }

      public DataPoint1D(double time, double value, double weight)
      {
         set(time, value, weight);
      }

      public void set(DataPoint1D other)
      {
         set(other.time, other.value, other.weight);
      }

      public void set(double time, double value, double weight)
      {
         this.time = time;
         this.value = value;
         this.weight = weight;
      }

      public void setTime(double time)
      {
         this.time = time;
      }

      public void setValue(double value)
      {
         this.value = value;
      }

      public void setWeight(double weight)
      {
         this.weight = weight;
      }

      @Override
      public double getTime()
      {
         return time;
      }

      @Override
      public double getValue()
      {
         return value;
      }

      @Override
      public double getWeight()
      {
         return weight;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getStringOf("(", ")", ",", time, value, weight);
      }
   }

   public interface DataPoint1DReadOnly
   {
      double getTime();

      double getValue();

      double getWeight();

      static DataPoint1DReadOnly link(DoubleSupplier timeProvider, DoubleSupplier valueProvider, DoubleSupplier weightProvider)
      {
         return new DataPoint1DReadOnly()
         {
            @Override
            public double getTime()
            {
               return timeProvider.getAsDouble();
            }

            @Override
            public double getValue()
            {
               return valueProvider.getAsDouble();
            }

            @Override
            public double getWeight()
            {
               return weightProvider.getAsDouble();
            }
         };
      }
   }
}