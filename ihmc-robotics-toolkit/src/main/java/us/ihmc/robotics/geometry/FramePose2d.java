package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class FramePose2d extends FrameGeometryObject<FramePose2d, Pose2D>
{
   private final Pose2D pose;

   public FramePose2d()
   {
      this(new FramePoint2D(), new FrameOrientation2d());
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

   public FramePose2d(FramePoint2D position, FrameOrientation2d orientation)
   {
      this(position.getReferenceFrame(), new Pose2D(position, orientation.getGeometryObject()));
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

   public void getPose(RigidBodyTransform rigidBodyTransformToPack)
   {
      pose.get(rigidBodyTransformToPack);
   }

   public void setOrientation(FrameOrientation2d orientation)
   {
      checkReferenceFrameMatch(orientation);
      pose.setOrientation(orientation.getGeometryObject());
   }

   public void setPosition(FramePoint2D position)
   {
      checkReferenceFrameMatch(position);
      pose.setPosition(position);
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

   public void getPositionIncludingFrame(FramePoint2D positionToPack)
   {
      positionToPack.setToZero(referenceFrame);
      pose.getPosition(positionToPack);
   }

   public void getOrientationIncludingFrame(FrameOrientation2d orientationToPack)
   {
      orientationToPack.setToZero(referenceFrame);
      pose.getOrientation(orientationToPack.getGeometryObject());
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
