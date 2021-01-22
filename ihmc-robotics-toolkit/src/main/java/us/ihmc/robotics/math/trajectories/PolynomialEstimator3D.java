package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.PositionTrajectoryGenerator;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalProvider;

public class PolynomialEstimator3D implements PositionTrajectoryGenerator, TimeIntervalProvider, Settable<PolynomialEstimator3D>
{
   private static final double regularization = 1e-5;
   private static final double defaultWeight = 1.0;

   private final TimeIntervalBasics timeInterval = new TimeIntervalBasics()
   {
      @Override
      public void setStartTime(double startTime)
      {
         xEstimator.getTimeInterval().setStartTime(startTime);
         yEstimator.getTimeInterval().setStartTime(startTime);
         zEstimator.getTimeInterval().setStartTime(startTime);
      }

      @Override
      public void setEndTime(double endTime)
      {
         xEstimator.getTimeInterval().setEndTime(endTime);
         yEstimator.getTimeInterval().setEndTime(endTime);
         zEstimator.getTimeInterval().setEndTime(endTime);
      }

      @Override
      public double getStartTime()
      {
         return xEstimator.getTimeInterval().getStartTime();
      }

      @Override
      public double getEndTime()
      {
         return xEstimator.getTimeInterval().getEndTime();
      }
   };

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

   private double currentTime = 0.0;

   private final PolynomialEstimator xEstimator = new PolynomialEstimator();
   private final PolynomialEstimator yEstimator = new PolynomialEstimator();
   private final PolynomialEstimator zEstimator = new PolynomialEstimator();


   public void reset()
   {
      currentTime = Double.NaN;
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
   public void set(PolynomialEstimator3D other)
   {
      getTimeInterval().set(other.getTimeInterval());
      xEstimator.set(other.xEstimator);
      yEstimator.set(other.yEstimator);
      zEstimator.set(other.zEstimator);
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   public void addObjectivePosition(double time, Tuple3DReadOnly value)
   {
      addObjectivePosition(defaultWeight, time, value);
   }

   public void addObjectivePosition(double weight, double time, Tuple3DReadOnly value)
   {
      xEstimator.addObjectivePosition(weight, time, value.getX());
      yEstimator.addObjectivePosition(weight, time, value.getY());
      zEstimator.addObjectivePosition(weight, time, value.getZ());
   }

   public void addObjectiveVelocity(double time, Tuple3DReadOnly value)
   {
      addObjectiveVelocity(defaultWeight, time, value);
   }

   public void addObjectiveVelocity(double weight, double time, Tuple3DReadOnly value)
   {
      xEstimator.addObjectiveVelocity(weight, time, value.getX());
      yEstimator.addObjectiveVelocity(weight, time, value.getY());
      zEstimator.addObjectiveVelocity(weight, time, value.getZ());
   }

   public void addConstraintPosition(double time, Tuple3DReadOnly value)
   {
      xEstimator.addConstraintPosition(time, value.getX());
      yEstimator.addConstraintPosition(time, value.getY());
      zEstimator.addConstraintPosition(time, value.getZ());
   }

   public void addConstraintVelocity(double time, Tuple3DReadOnly value)
   {
      xEstimator.addConstraintVelocity(time, value.getX());
      yEstimator.addConstraintVelocity(time, value.getY());
      zEstimator.addConstraintVelocity(time, value.getZ());
   }

   @Override
   public void initialize()
   {
      xEstimator.solve();
      yEstimator.solve();
      zEstimator.solve();
   }

   @Override
   public void compute(double time)
   {
      xEstimator.compute(time);
      yEstimator.compute(time);
      zEstimator.compute(time);
   }

   @Override
   public Point3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public Vector3DReadOnly getVelocity()
   {
      return velocity;
   }

   @Override
   public Vector3DReadOnly getAcceleration()
   {
      return acceleration;
   }

   @Override
   public boolean isDone()
   {
      return currentTime >= getTimeInterval().getEndTime();
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
