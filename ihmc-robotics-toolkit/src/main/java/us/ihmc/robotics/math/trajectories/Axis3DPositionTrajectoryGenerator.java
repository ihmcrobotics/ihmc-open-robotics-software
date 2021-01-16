package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.math.trajectories.interfaces.DoubleTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.PositionTrajectoryGenerator;

public abstract class Axis3DPositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final DoubleTrajectoryGenerator xTrajectory;
   private final DoubleTrajectoryGenerator yTrajectory;
   private final DoubleTrajectoryGenerator zTrajectory;

   private final Point3DReadOnly position = new Point3DReadOnly()
   {
      @Override
      public double getX()
      {
         return xTrajectory.getValue();
      }

      @Override
      public double getY()
      {
         return yTrajectory.getValue();
      }

      @Override
      public double getZ()
      {
         return zTrajectory.getValue();
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

   public Axis3DPositionTrajectoryGenerator(DoubleTrajectoryGenerator xTrajectory, DoubleTrajectoryGenerator yTrajectory, DoubleTrajectoryGenerator zTrajectory)
   {
      this.xTrajectory = xTrajectory;
      this.yTrajectory = yTrajectory;
      this.zTrajectory = zTrajectory;
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
}
