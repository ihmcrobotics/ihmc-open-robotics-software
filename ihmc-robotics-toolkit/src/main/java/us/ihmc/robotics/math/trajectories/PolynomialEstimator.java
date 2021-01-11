package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class PolynomialEstimator implements TimeIntervalBasics
{
   private static final double regularization = 1e-5;
   private static final double defaultWeight = 1.0;

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(0, 0);

   private final DMatrixRMaj hessian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj gradient = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj coefficients = new DMatrixRMaj(0, 0);

   private int order = 0;

   private double tInitial;
   private double tFinal;

   private double position;
   private double velocity;
   private double acceleration;

   public int getOrder()
   {
      return order;
   }

   @Override
   public void reset()
   {
      tInitial = Double.NaN;
      tFinal = Double.NaN;

      position = Double.NaN;
      velocity = Double.NaN;
      acceleration = Double.NaN;
   }

   public void reshape(int order)
   {
      this.order = order;

      hessian.reshape(order, order);
      gradient.reshape(order, 1);
      coefficients.reshape(order, 1);

      hessian.zero();
      gradient.zero();
      coefficients.zero();
   }

   @Override
   public void setEndTime(double tFinal)
   {
      this.tFinal = tFinal;
   }

   @Override
   public void setStartTime(double t0)
   {
      this.tInitial = t0;
   }

   @Override
   public double getEndTime()
   {
      return this.tFinal;
   }

   @Override
   public double getStartTime()
   {
      return this.tInitial;
   }

   public void addObjectivePosition(double time, double value)
   {
      addObjectivePosition(defaultWeight, time, value);
   }

   public void addObjectivePosition(double weight, double time, double value)
   {
      time -= getStartTime();
      double timeI = 1.0;
      for (int i = 0; i < order; i++)
      {
         hessian.add(i, i, weight * timeI * timeI);
         gradient.add(i, 0, weight * value * timeI);

         double timeJ = timeI * time;
         for (int j = i + 1; j < order; j++)
         {
            double hessianEntrant = weight * timeI * timeJ;
            hessian.add(i, j, hessianEntrant);
            hessian.add(j, i, hessianEntrant);

            timeJ *= time;
         }

         timeI *= time;
      }
   }

   public void solve()
   {
      MatrixTools.addDiagonal(hessian, regularization);

      solver.setA(hessian);
      solver.solve(gradient, coefficients);
   }

   public void compute(double time)
   {
      position = 0.0;
      velocity = 0.0;
      acceleration = 0.0;

      time -= getStartTime();
      double timeI = 1.0;

      int idx = 0;
      for (; idx < order - 2; idx++)
      {
         position += coefficients.get(idx) * timeI;
         velocity += (idx + 1) * coefficients.get(idx + 1) * timeI;
         acceleration += (idx + 2) * (idx + 1) * coefficients.get(idx + 2) * timeI;

         timeI *= time;
      }

      for (; idx < order - 1; idx++)
      {
         position += coefficients.get(idx) * timeI;
         velocity += (idx + 1) * coefficients.get(idx + 1) * timeI;

         timeI *= time;
      }

      for (; idx < order; idx++)
      {
         position += coefficients.get(idx) * timeI;
         timeI *= time;
      }
   }

   public DMatrixRMaj getCoefficients()
   {
      return coefficients;
   }

   public double getPosition()
   {
      return position;
   }

   public double getVelocity()
   {
      return velocity;
   }

   public double getAcceleration()
   {
      return acceleration;
   }
}
