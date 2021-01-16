package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public abstract class Axis3DPositionTrajectoryGenerator implements FramePositionTrajectoryGenerator
{
   private final DoubleTrajectoryGenerator xTrajectory;
   private final DoubleTrajectoryGenerator yTrajectory;
   private final DoubleTrajectoryGenerator zTrajectory;

   private final FramePoint3DReadOnly position = new FramePoint3DReadOnly()
   {
      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }

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

   private final FrameVector3DReadOnly velocity = new FrameVector3DReadOnly()
   {
      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }

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

   private final FrameVector3DReadOnly acceleration = new FrameVector3DReadOnly()
   {
      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return ReferenceFrame.getWorldFrame();
      }

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
   public FramePoint3DReadOnly getPosition()
   {
      return position;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return velocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return acceleration;
   }
}
