package us.ihmc.quadrupedRobotics.planning.trajectory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import us.ihmc.commons.Epsilons;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.Trajectory;

public class ExponentialAndCubicTrajectory3D
{
   private final ExponentialAndCubicTrajectory xTrajectory = new ExponentialAndCubicTrajectory();
   private final ExponentialAndCubicTrajectory yTrajectory = new ExponentialAndCubicTrajectory();
   private final ExponentialAndCubicTrajectory zTrajectory = new ExponentialAndCubicTrajectory();
   private final ExponentialAndCubicTrajectory[] trajectories;

   private final Point3DReadOnly position = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getPosition();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getPosition();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getPosition();
      }

      @Override
      public String toString()
      {
         return "X: " + getX() + " Y: " + getY() + " Z: " + getZ();
      }
   };

   private final Vector3DReadOnly velocity = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getVelocity();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getVelocity();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getVelocity();
      }
   };

   private final Vector3DReadOnly acceleration = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getAcceleration();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getAcceleration();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getAcceleration();
      }
   };

   public ExponentialAndCubicTrajectory3D()
   {
      reset();
      trajectories = new ExponentialAndCubicTrajectory[] {xTrajectory, yTrajectory, zTrajectory};
   }

   public void compute(double t)
   {
      xTrajectory.compute(t);
      yTrajectory.compute(t);
      zTrajectory.compute(t);
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

   public void reset()
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).reset();
   }

   private ExponentialAndCubicTrajectory getTrajectory(int index)
   {
      return trajectories[index];
   }

   public void setTime(double t0, double tFinal)
   {
      setInitialTime(t0);
      setFinalTime(tFinal);
   }

   public void setTimeConstant(double timeConstant)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setTimeConstant(timeConstant);
   }

   public void setFinalTime(double tFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setFinalTime(tFinal);
   }

   public void setInitialTime(double t0)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setInitialTime(t0);
   }

   public double getDuration()
   {
      return getFinalTime() - getInitialTime();
   }

   public double getInitialTime()
   {
      if (MathTools.epsilonCompare(xTrajectory.getInitialTime(), yTrajectory.getInitialTime(), Epsilons.ONE_THOUSANDTH)
            && MathTools.epsilonCompare(xTrajectory.getInitialTime(), zTrajectory.getInitialTime(), Epsilons.ONE_THOUSANDTH))
         return xTrajectory.getInitialTime();
      else
      {
         //PrintTools.warn("Trajectory initial times do not match. Using X trajectory times for computation");
         return xTrajectory.getInitialTime();
      }
   }

   public double getFinalTime()
   {
      if (MathTools.epsilonCompare(xTrajectory.getFinalTime(), yTrajectory.getFinalTime(), Epsilons.ONE_THOUSANDTH)
            && MathTools.epsilonCompare(xTrajectory.getFinalTime(), zTrajectory.getFinalTime(), Epsilons.ONE_THOUSANDTH))
         return xTrajectory.getFinalTime();
      else
      {
         //PrintTools.warn("Trajectory final times do not match. Using X trajectory times for computation");
         return xTrajectory.getFinalTime();
      }
   }


   public boolean timeIntervalContains(double timeToCheck, double epsilon)
   {
      return (xTrajectory.timeIntervalContains(timeToCheck, epsilon) && yTrajectory.timeIntervalContains(timeToCheck, epsilon)
            && zTrajectory.timeIntervalContains(timeToCheck, epsilon));
   }

   public boolean timeIntervalContains(double timeToCheck)
   {
      return (xTrajectory.timeIntervalContains(timeToCheck) && yTrajectory.timeIntervalContains(timeToCheck) && zTrajectory.timeIntervalContains(timeToCheck));
   }

   public void setFromBounds(double t0, double tFinal, double timeConstant, Point3DReadOnly x0, Vector3DReadOnly xd0, Vector3DReadOnly vrp0,
                             Point3DReadOnly xFinal, Vector3DReadOnly xdFinal, Vector3DReadOnly vrpFinal)
   {
      for (int index = 0; index < 3; index++)
         getTrajectory(index).setFromBounds(t0, tFinal, timeConstant, x0.getElement(index), xd0.getElement(index), vrp0.getElement(index),
                                            xFinal.getElement(index), xdFinal.getElement(index), vrpFinal.getElement(index));
   }




}
