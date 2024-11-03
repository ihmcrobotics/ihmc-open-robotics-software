package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public class PolynomialEstimator implements TimeIntervalProvider, Settable<PolynomialEstimator>
{
   private static final boolean debug = false;

   private static final double regularization = 1e-5;
   private static final double defaultWeight = 1.0;

   private final LinearSolverDense<DMatrixRMaj> solver = LinearSolverFactory_DDRM.general(0, 0);

   private final DMatrixRMaj hessian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj gradient = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj equalityConstraintJacobian = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj equalityConstraintJacobianTranspose = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj equalityConstraintObjective = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj A = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj b = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj coefficientsAndLagrangeMultipliers = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj coefficients = new DMatrixRMaj(0, 0);

   private int order = 0;

   private final TimeIntervalBasics timeIntervalBasics = new TimeInterval(0.0, Double.POSITIVE_INFINITY);

   private double position = Double.NaN;
   private double velocity = Double.NaN;
   private double acceleration = Double.NaN;

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeIntervalBasics;
   }

   @Override
   public void set(PolynomialEstimator other)
   {
      getTimeInterval().set(other.getTimeInterval());
      reshape(other.getOrder());

      coefficients.set(other.coefficients);
   }

   public int getOrder()
   {
      return order;
   }

   public void reset()
   {
      getTimeInterval().setInterval(0.0, Double.POSITIVE_INFINITY);

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

      equalityConstraintJacobian.reshape(0, order);
      equalityConstraintObjective.reshape(0, 0);

      hessian.zero();
      gradient.zero();
      coefficients.zero();

      equalityConstraintJacobian.zero();
      equalityConstraintObjective.zero();
   }

   public void addObjectivePosition(double time, double value)
   {
      addObjectivePosition(defaultWeight, time, value);
   }

   public void addObjectivePosition(double weight, double time, double value)
   {
      time -= getTimeInterval().getStartTime();
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

      if (debug && MatrixTools.containsNaN(hessian))
         throw new RuntimeException("Error.");
      if (debug && MatrixTools.containsNaN(gradient))
         throw new RuntimeException("Error.");
   }

   public void addObjectiveVelocity(double time, double value)
   {
      addObjectiveVelocity(defaultWeight, time, value);
   }

   public void addObjectiveVelocity(double weight, double time, double value)
   {
      time -= getTimeInterval().getStartTime();
      double timeI = 1.0;
      for (int idx = 0; idx < order - 1; idx++)
      {
         int row = idx + 1;

         hessian.add(row, row, weight * row * row * timeI * timeI);
         gradient.add(row, 0, weight * value * row * timeI);

         double timeJ = timeI * time;
         for (int j = idx + 1; j < order - 1; j++)
         {
            int col = j + 1;
            double hessianEntrant = weight * row * col * timeI * timeJ;
            hessian.add(row, col, hessianEntrant);
            hessian.add(col, row, hessianEntrant);

            timeJ *= time;
         }

         timeI *= time;
      }

      if (debug && MatrixTools.containsNaN(hessian))
         throw new RuntimeException("Error.");
      if (debug && MatrixTools.containsNaN(gradient))
         throw new RuntimeException("Error.");
   }

   public void addObjectiveAcceleration(double time, double value)
   {
      addObjectiveAcceleration(defaultWeight, time, value);
   }

   public void addObjectiveAcceleration(double weight, double time, double value)
   {
      time -= getTimeInterval().getStartTime();
      double timeI = 1.0;
      for (int idx = 0; idx < order - 2; idx++)
      {
         int row = idx + 2;
         int prefixI = row * (row - 1);

         hessian.add(row, row, weight * prefixI * prefixI * timeI * timeI);
         gradient.add(row, 0, weight * value * prefixI * timeI);

         double timeJ = timeI * time;
         for (int j = idx + 1; j < order - 2; j++)
         {
            int col = j + 2;
            int prefixJ = col * (col - 1);
            double hessianEntrant = weight * prefixI * prefixJ * timeI * timeJ;
            hessian.add(row, col, hessianEntrant);
            hessian.add(col, row, hessianEntrant);

            timeJ *= time;
         }

         timeI *= time;
      }

      if (debug && MatrixTools.containsNaN(hessian))
         throw new RuntimeException("Error.");
      if (debug && MatrixTools.containsNaN(gradient))
         throw new RuntimeException("Error.");
   }

   public void addConstraintPosition(double time, double value)
   {
      int previousSize = equalityConstraintObjective.getNumRows();
      int problemSize = equalityConstraintJacobian.getNumCols();

      equalityConstraintJacobian.reshape(previousSize + 1, problemSize, true);
      equalityConstraintObjective.reshape(previousSize + 1, 1, true);

      double timeI = 1.0;
      for (int idx = 0; idx < order; idx++)
      {
         equalityConstraintJacobian.set(previousSize, idx, timeI);
         timeI *= time;
      }

      equalityConstraintObjective.set(previousSize, 0, value);
   }

   public void addConstraintVelocity(double time, double value)
   {
      int previousSize = equalityConstraintObjective.getNumRows();
      int problemSize = equalityConstraintJacobian.getNumCols();

      equalityConstraintJacobian.reshape(previousSize + 1, problemSize, true);
      equalityConstraintObjective.reshape(previousSize + 1, 1, true);

      double timeI = 1.0;
      for (int idx = 1; idx < order; idx++)
      {
         equalityConstraintJacobian.set(previousSize, idx, idx * timeI);
         timeI *= time;
      }

      equalityConstraintObjective.set(previousSize, 0, value);
   }

   public void addConstraintAcceleration(double time, double value)
   {
      int previousSize = equalityConstraintObjective.getNumRows();
      int problemSize = equalityConstraintJacobian.getNumCols();

      equalityConstraintJacobian.reshape(previousSize + 1, problemSize, true);
      equalityConstraintObjective.reshape(previousSize + 1, 1, true);

      double timeI = 1.0;
      for (int idx = 2; idx < order; idx++)
      {
         equalityConstraintJacobian.set(previousSize, idx, idx * (idx - 1) * timeI);
         timeI *= time;
      }

      equalityConstraintObjective.set(previousSize, 0, value);
   }

   public void solve()
   {
      MatrixTools.addDiagonal(hessian, regularization);

      int constraints = equalityConstraintObjective.getNumRows();
      A.reshape(order + constraints, order + constraints);
      b.reshape(order + constraints, 1);
      coefficientsAndLagrangeMultipliers.reshape(order + constraints, 1);

      A.zero();
      b.zero();

      equalityConstraintJacobianTranspose.reshape(order, constraints);
      CommonOps_DDRM.transpose(equalityConstraintJacobian, equalityConstraintJacobianTranspose);

      MatrixTools.setMatrixBlock(A, 0, 0, hessian, 0, 0, order, order, 1.0);
      MatrixTools.setMatrixBlock(A, 0, order, equalityConstraintJacobianTranspose, 0, 0, order, constraints, 1.0);
      MatrixTools.setMatrixBlock(A, order, 0, equalityConstraintJacobian, 0, 0, constraints, order, 1.0);

      MatrixTools.setMatrixBlock(b, 0, 0, gradient, 0, 0, order, 1, 1.0);
      MatrixTools.setMatrixBlock(b, order, 0, equalityConstraintObjective, 0, 0, constraints, 1, 1.0);

      solver.setA(A);
      solver.solve(b, coefficientsAndLagrangeMultipliers);

      if (debug && MatrixTools.containsNaN(coefficientsAndLagrangeMultipliers))
         throw new RuntimeException("Error.");
      MatrixTools.setMatrixBlock(coefficients, 0, 0, coefficientsAndLagrangeMultipliers, 0, 0, order, 1, 1.0);
   }

   public void compute(double time)
   {
      position = 0.0;
      velocity = 0.0;
      acceleration = 0.0;

      time -= getTimeInterval().getStartTime();
      time = Math.min(time, getTimeInterval().getDuration());
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
