package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoOptimizedPolynomial
{
   private final int maximumNumberOfCoefficients;
   private double pos, vel, acc, jerk;
   private final double[] xPowers;

   private final RecyclingArrayList<PositionPoint1D> positionPoints = new RecyclingArrayList<>(PositionPoint1D::new);
   private final RecyclingArrayList<VelocityPoint1D> velocityPoints = new RecyclingArrayList<>(VelocityPoint1D::new);
   private final YoDouble[] coefficients;
   private final YoInteger numberOfCoefficients;

   private final DMatrixRMaj H;
   private final DMatrixRMaj Haccel;
   private final DMatrixRMaj Hjerk;
   private final DMatrixRMaj g;
   private final DMatrixRMaj x;

   private final DMatrixRMaj jacobian;
   private final DMatrixRMaj weight;
   private final DMatrixRMaj objective;

   private final DMatrixRMaj jtW;

   private final LinearSolverDense<DMatrixRMaj> solver;

   private double regularizationWeight = 1.0e-8;
   private double defaultPointWeight = 1.0;
   private double accelerationMinimizationWeight = Double.NaN;
   private double jerkMinimizationWeight = Double.NaN;

   private double minX = Double.POSITIVE_INFINITY;
   private double maxX = Double.NEGATIVE_INFINITY;

   public YoOptimizedPolynomial(String name, int maximumNumberOfCoefficients, YoRegistry registry)
   {
      this.maximumNumberOfCoefficients = maximumNumberOfCoefficients;

      solver = LinearSolverFactory_DDRM.symmPosDef(maximumNumberOfCoefficients);

      coefficients = new YoDouble[maximumNumberOfCoefficients];

      H = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      Haccel = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      Hjerk = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      g = new DMatrixRMaj(maximumNumberOfCoefficients, 1);
      x = new DMatrixRMaj(maximumNumberOfCoefficients, 1);

      jacobian = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      weight = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);
      objective = new DMatrixRMaj(maximumNumberOfCoefficients, 1);

      xPowers = new double[maximumNumberOfCoefficients];

      jtW = new DMatrixRMaj(maximumNumberOfCoefficients, maximumNumberOfCoefficients);

      numberOfCoefficients = new YoInteger(name + "_nCoeffs", registry);
      numberOfCoefficients.set(maximumNumberOfCoefficients);

      for (int i = 0; i < maximumNumberOfCoefficients; i++)
      {
         coefficients[i] = new YoDouble(name + "_a" + i, registry);
      }
   }

   public void reshape(int numberOfCoefficientsRequired)
   {
      if (numberOfCoefficientsRequired > maximumNumberOfCoefficients)
         throw new RuntimeException("Maximum number of coefficients is: " + maximumNumberOfCoefficients + ", can't build the polynomial as it requires: "
                                    + numberOfCoefficientsRequired + " coefficients.");

      H.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      Haccel.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      Hjerk.reshape(numberOfCoefficientsRequired, numberOfCoefficientsRequired);
      g.reshape(numberOfCoefficientsRequired, 1);
      x.reshape(numberOfCoefficientsRequired, 1);

      numberOfCoefficients.set(numberOfCoefficientsRequired);

      for (int i = numberOfCoefficientsRequired; i < maximumNumberOfCoefficients; i++)
         coefficients[i].set(Double.NaN);
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

   public void setAccelerationMinimizationWeight(double weight)
   {
      this.accelerationMinimizationWeight = weight;
   }

   public void setJerkMinimizationWeight(double weight)
   {
      this.jerkMinimizationWeight = weight;
   }

   public void clear()
   {
      positionPoints.clear();
      velocityPoints.clear();
   }

   public void addPositionPoint(PositionPoint1D point)
   {
      positionPoints.add().set(point);
   }

   public void addPositionPoint(double x, double y)
   {
      addPositionPoint(x, y, defaultPointWeight);
   }

   public void addPositionPoint(double x, double y, double weight)
   {
      positionPoints.add().set(x, y, weight);
   }

   public void addVelocityPoint(VelocityPoint1D point)
   {
      velocityPoints.add().set(point);
   }

   public void addVelocityPoint(double x, double yDot)
   {
      addVelocityPoint(x, yDot, defaultPointWeight);
   }

   public void addVelocityPoint(double x, double yDot, double weight)
   {
      velocityPoints.add().set(x, yDot, weight);
   }

   public void fit()
   {
      minX = Double.POSITIVE_INFINITY;
      maxX = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < positionPoints.size(); i++)
      {
         PositionPoint1D point = positionPoints.get(i);

         minX = Math.min(minX, point.getX());
         maxX = Math.max(maxX, point.getX());
      }

      for (int i = 0; i < velocityPoints.size(); i++)
      {
         VelocityPoint1D point = velocityPoints.get(i);

         minX = Math.min(minX, point.getX());
         maxX = Math.max(maxX, point.getX());
      }

      int nCoeffs = numberOfCoefficients.getIntegerValue();
      int size = positionPoints.size() + velocityPoints.size();
      jacobian.reshape(size, nCoeffs);
      weight.reshape(size, size);
      objective.reshape(size, 1);
      jacobian.zero();
      weight.zero();
      objective.zero();

      for (int i = 0; i < positionPoints.size(); i++)
         setPositionConstraint(i, positionPoints.get(i));
      for (int i = 0; i < velocityPoints.size(); i++)
         setVelocityConstraint(i + positionPoints.size(), velocityPoints.get(i));

      jtW.reshape(nCoeffs, size);
      CommonOps_DDRM.multTransA(jacobian, weight, jtW);

      CommonOps_DDRM.mult(jtW, jacobian, H);
      CommonOps_DDRM.mult(jtW, objective, g);

      if (accelerationMinimizationWeight > 0.0)
      {
         computeCostHessianOfIntegralSquared(Haccel, 2);
         CommonOps_DDRM.addEquals(H, accelerationMinimizationWeight, Haccel);
      }
      else
      {
         Haccel.zero();
      }

      if (jerkMinimizationWeight > 0.0)
      {
         computeCostHessianOfIntegralSquared(Hjerk, 3);
         CommonOps_DDRM.addEquals(H, jerkMinimizationWeight, Hjerk);
      }
      else
      {
         Hjerk.zero();
      }

      for (int i = 0; i < H.getNumCols(); i++)
      {
         H.add(i, i, regularizationWeight);
      }


      solver.setA(H);
      solver.solve(g, x);

      for (int i = 0; i < numberOfCoefficients.getValue(); i++)
      {
         coefficients[i].set(x.get(i));
      }
   }

   public void compute(double x)
   {
      double time = normalizeTime(x);
      setXPowers(xPowers, time);

      pos = vel = acc = jerk = 0.0;
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         pos += coefficients[i].getDoubleValue() * xPowers[i];
      }

      for (int i = 1; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         vel += i * coefficients[i].getDoubleValue() * xPowers[i - 1];
      }

      for (int i = 2; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         acc += (i - 1) * i * coefficients[i].getDoubleValue() * xPowers[i - 2];
      }

      for (int i = 3; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         jerk += (i - 2) * (i - 1) * i * coefficients[i].getDoubleValue() * xPowers[i - 3];
      }
   }

   public double getPosition()
   {
      return pos;
   }

   public double getVelocity()
   {
      return vel;
   }

   public double getAcceleration()
   {
      return acc;
   }

   public double getJerk()
   {
      return jerk;
   }

   public double getCoefficient(int i)
   {
      return coefficients[i].getDoubleValue();
   }

   public void getCoefficients(double[] coefficients)
   {
      for (int i = 0; i < this.coefficients.length; i++)
      {
         coefficients[i] = this.coefficients[i].getDoubleValue();
      }
   }

   /**
    * To be used only for testing. Garbage creating function
    */
   public double[] getCoefficients()
   {
      double[] ret = new double[numberOfCoefficients.getIntegerValue()];
      getCoefficients(ret);
      return ret;
   }

   /**
    * Returns the order-th derivative of the polynomial at x
    */
   public double getDerivative(int order, double x)
   {
      double time = normalizeTime(x);
      setXPowers(xPowers, time);

      double dPos = 0.0;
      for (int i = order; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         dPos += getDerivativeCoefficient(order, i) * coefficients[i].getDoubleValue() * xPowers[i - order];
      }
      return dPos;
   }

   /**
    * Returns the constant coefficient at the exponent-th entry of the order-th derivative vector
    * Example: order = 4, exponent = 5 ==> returns 5*4*3*2
    */
   public int getDerivativeCoefficient(int order, int exponent)
   {
      int coeff = 1;
      for (int i = exponent; i > exponent - order; i--)
      {
         coeff *= i;
      }
      return coeff;
   }

   /**
    * Returns the order-th derivative of the xPowers vector at value x (Note: does NOT return the
    * YoPolynomials order-th derivative at x)
    */
   public void getXPowersDerivativeVector(int order, double x, DMatrixRMaj xPowersDerivativeVectorToPack)
   {
      double time = normalizeTime(x);
      setXPowers(xPowers, time);
      xPowersDerivativeVectorToPack.zero();

      for (int i = order; i < numberOfCoefficients.getIntegerValue(); i++)
         xPowersDerivativeVectorToPack.set(i, getDerivativeCoefficient(order, i) * xPowers[i - order]);
   }

   private void setPositionConstraint(int row, PositionPoint1D point)
   {
      setConstraintRow(row, point.getX(), point.getY(), point.getWeight(), 0);
   }

   private void setVelocityConstraint(int row, VelocityPoint1D point)
   {
      setConstraintRow(row, point.getX(), point.getYDot(), point.getWeight(), 1);
   }

   private void setConstraintRow(int row, double x, double desiredYDerivative, double weight, int derivativeOrderWithPositionBeingZero)
   {
      double x_n = 1.0;
      double time = normalizeTime(x);

      for (int col = derivativeOrderWithPositionBeingZero; col < numberOfCoefficients.getIntegerValue(); col++)
      {
         double columnPower = 1.0;
         for (int i = 0; i < derivativeOrderWithPositionBeingZero; i++)
         {
            columnPower *= col - i;
         }
         jacobian.set(row, col, x_n * columnPower);
         x_n *= time;
      }

      this.weight.set(row, row, weight);
      objective.set(row, desiredYDerivative);
   }

   /**
    * Used to compute the hessian of the cost function defined by the integral of the squared value of the derivative function
    */
   void computeCostHessianOfIntegralSquared(DMatrixRMaj hessianToPack, int derivativeToIntegrate)
   {
      for (int i = 0; i < numberOfCoefficients.getIntegerValue(); i++)
      {
         for (int j = 0; j < numberOfCoefficients.getIntegerValue(); j++)
         {
            if (i >= derivativeToIntegrate && j >= derivativeToIntegrate)
            {
               int power = i + j - 2 * derivativeToIntegrate + 1;
               double scalar = 1.0 / power;

               for (int k = 0; k < derivativeToIntegrate; k++)
               {
                  scalar *= (i - k) * (j - k);
               }

               double value = scalar * (MathTools.pow(maxX, power) - MathTools.pow(minX, power));
               hessianToPack.set(i, j, value);
            }
            else
            {
               hessianToPack.set(i, j, 0.0);
            }
         }
      }
   }

   public double normalizeTime(double time)
   {
      double timeScale = timeScale();
      if (timeScale > 1e-5)
         return (time - minX) / timeScale;
      else
         return time;
   }

   private double timeScale()
   {
      return maxX - minX;
   }


   private static void setXPowers(double[] xPowers, double x)
   {
      xPowers[0] = 1.0;
      for (int i = 1; i < xPowers.length; i++)
      {
         xPowers[i] = xPowers[i - 1] * x;
      }
   }

   public static class PositionPoint1D
   {
      private double x;
      private double y;
      private double weight = 1.0;

      public PositionPoint1D()
      {
      }

      public void set(PositionPoint1D other)
      {
         set(other.x, other.y, other.weight);
      }

      public void set(double x, double y, double weight)
      {
         this.x = x;
         this.y = y;
         this.weight = weight;
      }

      public void setX(double x)
      {
         this.x = x;
      }

      public void setY(double y)
      {
         this.y = y;
      }

      public void setWeight(double weight)
      {
         this.weight = weight;
      }

      public double getX()
      {
         return x;
      }

      public double getY()
      {
         return y;
      }

      public double getWeight()
      {
         return weight;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getStringOf("(", ")", ",", x, y, weight);
      }
   }

   public static class VelocityPoint1D
   {
      private double x;
      private double yDot;
      private double weight = 1.0;

      public VelocityPoint1D()
      {
      }

      public void set(VelocityPoint1D other)
      {
         set(other.x, other.yDot, other.weight);
      }

      public void set(double x, double yDot, double weight)
      {
         this.x = x;
         this.yDot = yDot;
         this.weight = weight;
      }

      public void setX(double x)
      {
         this.x = x;
      }

      public void setYDot(double yDot)
      {
         this.yDot = yDot;
      }

      public void setWeight(double weight)
      {
         this.weight = weight;
      }

      public double getX()
      {
         return x;
      }

      public double getYDot()
      {
         return yDot;
      }

      public double getWeight()
      {
         return weight;
      }

      @Override
      public String toString()
      {
         return EuclidCoreIOTools.getStringOf("(", ")", ",", x, yDot, weight);
      }
   }
}
