package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.LinearSolverFactory_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.time.TimeIntervalBasics;

public class PolynomialEstimator3D implements Trajectory3DReadOnly, TimeIntervalBasics
{
   private static final double regularization = 1e-5;
   private static final double defaultWeight = 1.0;

   private final Point3DReadOnly position = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xEstimator.getPosition();
      }

      @Override
      public double getY()
      {
         return yEstimator.getPosition();
      }

      @Override
      public double getZ()
      {
         return zEstimator.getPosition();
      }
   };

   private final Vector3DReadOnly velocity = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xEstimator.getVelocity();
      }

      @Override
      public double getY()
      {
         return yEstimator.getVelocity();
      }

      @Override
      public double getZ()
      {
         return zEstimator.getVelocity();
      }
   };

   private final Vector3DReadOnly acceleration = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xEstimator.getAcceleration();
      }

      @Override
      public double getY()
      {
         return yEstimator.getAcceleration();
      }

      @Override
      public double getZ()
      {
         return zEstimator.getAcceleration();
      }
   };

   private final PolynomialEstimator xEstimator = new PolynomialEstimator();
   private final PolynomialEstimator yEstimator = new PolynomialEstimator();
   private final PolynomialEstimator zEstimator = new PolynomialEstimator();

   @Override
   public void reset()
   {
      xEstimator.reset();
      yEstimator.reset();
      zEstimator.reset();
   }

   public void reshape(int order)
   {
      xEstimator.reshape(order);
      yEstimator.reshape(order);
      zEstimator.reshape(order);
   }

   @Override
   public void setEndTime(double tFinal)
   {
      xEstimator.setEndTime(tFinal);
      yEstimator.setEndTime(tFinal);
      zEstimator.setEndTime(tFinal);
   }

   @Override
   public void setStartTime(double t0)
   {
      xEstimator.setStartTime(t0);
      yEstimator.setStartTime(t0);
      zEstimator.setStartTime(t0);
   }

   @Override
   public double getEndTime()
   {
      return xEstimator.getEndTime();
   }

   @Override
   public double getStartTime()
   {
      return xEstimator.getStartTime();
   }

   public void addObjectivePosition(double time, Point3DReadOnly value)
   {
      addObjectivePosition(defaultWeight, time, value);
   }

   public void addObjectivePosition(double weight, double time, Point3DReadOnly value)
   {
      xEstimator.addObjectivePosition(weight, time, value.getX());
      yEstimator.addObjectivePosition(weight, time, value.getY());
      zEstimator.addObjectivePosition(weight, time, value.getZ());
   }

   public void addObjectiveVelocity(double time, Vector3DReadOnly value)
   {
      addObjectiveVelocity(defaultWeight, time, value);
   }

   public void addObjectiveVelocity(double weight, double time, Vector3DReadOnly value)
   {
      xEstimator.addObjectiveVelocity(weight, time, value.getX());
      yEstimator.addObjectiveVelocity(weight, time, value.getY());
      zEstimator.addObjectiveVelocity(weight, time, value.getZ());
   }

   public void solve()
   {
      xEstimator.solve();
      yEstimator.solve();
      zEstimator.solve();
   }

   public void compute(double time)
   {
      xEstimator.compute(time);
      yEstimator.compute(time);
      zEstimator.compute(time);
   }

   public Point3DReadOnly getPosition()
   {
      return position;
   }

   public Vector3DReadOnly getVelocity()
   {
      return velocity;
   }

   public Vector3DReadOnly getAcceleration()
   {
      return acceleration;
   }
}
