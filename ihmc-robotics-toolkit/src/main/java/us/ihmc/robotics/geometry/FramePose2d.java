package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameOrientation2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FramePose2d extends FrameGeometryObject<FramePose2d, Pose2D>
{
   private final Pose2D pose;

   private final FixedFrameOrientation2DBasics orientationPart = new FixedFrameOrientation2DBasics()
   {
      @Override
      public void setYaw(double yaw)
      {
         pose.setYaw(yaw);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getYaw()
      {
         return pose.getYaw();
      }

      @Override
      public void applyTransform(Transform transform)
      {
         pose.getOrientation().applyTransform(transform);
      }

      @Override
      public void applyInverseTransform(Transform transform)
      {
         pose.getOrientation().applyInverseTransform(transform);
      }
   };

   private final FixedFramePoint2DBasics positionPart = new FixedFramePoint2DBasics()
   {
      @Override
      public void setX(double x)
      {
         pose.setX(x);
      }

      @Override
      public void setY(double y)
      {
         pose.setY(y);
      }

      @Override
      public ReferenceFrame getReferenceFrame()
      {
         return referenceFrame;
      }

      @Override
      public double getX()
      {
         return pose.getX();
      }

      @Override
      public double getY()
      {
         return pose.getY();
      }

      @Override
      public void applyTransform(Transform transform, boolean checkIfTransformInXYplane)
      {
         pose.getPosition().applyTransform(transform, checkIfTransformInXYplane);
      }

      @Override
      public void applyInverseTransform(Transform transform, boolean checkIfTransformInXYplane)
      {
         pose.getPosition().applyInverseTransform(transform, checkIfTransformInXYplane);
      }
   };

   public FramePose2d()
   {
      this(new FramePoint2D(), new FrameOrientation2D());
   }

   public FramePose2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, new Pose2D());
   }

   public FramePose2d(ReferenceFrame referenceFrame, Pose2D pose)
   {
      super(referenceFrame, pose);
      this.pose = getGeometryObject();
   }

   public FramePose2d(FramePoint2D position, FrameOrientation2D orientation)
   {
      this(position.getReferenceFrame(), new Pose2D(position, orientation));
      position.checkReferenceFrameMatch(orientation);
   }

   public FramePose2d(ReferenceFrame referenceFrame, Point2DReadOnly position, double orientation)
   {
      this(referenceFrame, new Pose2D(position.getX(), position.getY(), orientation));
   }

   public FramePose2d(FramePose2d pose)
   {
      this(pose.getReferenceFrame(), new Pose2D(pose.getGeometryObject()));
   }

   public FramePose2d(FramePose pose)
   {
      this(pose.getReferenceFrame(), new Pose2D(pose.getGeometryObject()));
   }

   public void get(RigidBodyTransform rigidBodyTransformToPack)
   {
      pose.get(rigidBodyTransformToPack);
   }

   public FixedFramePoint2DBasics getPosition()
   {
      return positionPart;
   }

   public FixedFrameOrientation2DBasics getOrientation()
   {
      return orientationPart;
   }

   public void setOrientation(FrameOrientation2D orientation)
   {
      checkReferenceFrameMatch(orientation);
      pose.setOrientation(orientation);
   }

   public void setPosition(FramePoint2D position)
   {
      checkReferenceFrameMatch(position);
      pose.setPosition(position);
   }
   
   public void setIncludingFrame(FramePose framePose)
   {
      referenceFrame = framePose.getReferenceFrame();
      pose.set(framePose.getGeometryObject());
   }
   
   public void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform)
   {
      setIncludingFrame(referenceFrame, rigidBodyTransform, true);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform, boolean checkIsTransform2D)
   {
      setToZero(referenceFrame);
      pose.set(rigidBodyTransform, checkIsTransform2D);
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double yaw)
   {
      setToZero(referenceFrame);
      pose.set(x, y, yaw);
   }

   public void setX(double x)
   {
      pose.setX(x);
   }

   public void setY(double y)
   {
      pose.setY(y);
   }

   public void setYaw(double yaw)
   {
      pose.setYaw(yaw);
   }

   public double getYaw()
   {
      return pose.getYaw();
   }

   public double getX()
   {
      return pose.getX();
   }

   public double getY()
   {
      return pose.getY();
   }

   public void interpolate(FramePose2d framePose1, FramePose2d framePose2, double alpha)
   {
      checkReferenceFrameMatch(framePose1);
      framePose1.checkReferenceFrameMatch(framePose2);
      pose.interpolate(framePose1.pose, framePose2.pose, alpha);
   }

   public double getPositionDistance(FramePose2d other)
   {
      checkReferenceFrameMatch(other);
      return pose.getPositionDistance(other.pose);
   }

   public double getOrientationDistance(FramePose2d other)
   {
      checkReferenceFrameMatch(other);
      return pose.getOrientationDistance(other.pose);
   }
}
