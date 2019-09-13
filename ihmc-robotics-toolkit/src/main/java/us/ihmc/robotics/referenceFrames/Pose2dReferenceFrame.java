package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.FrameOrientation2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class Pose2dReferenceFrame extends ReferenceFrame
{
   private final FramePose2D originPose;

   public Pose2dReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      originPose = new FramePose2D(parentFrame);
   }

   public Pose2dReferenceFrame(String frameName, FramePose2D pose)
   {
      this(frameName, pose.getReferenceFrame());
      setPoseAndUpdate(pose);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      originPose.get(transformToParent);
   }

   public void setPositionAndUpdate(FramePoint2D framePoint)
   {
      framePoint.checkReferenceFrameMatch(getParent());
      originPose.setPosition(framePoint);
      this.update();
   }

   public void setOrientationAndUpdate(FrameOrientation2D frameOrientation)
   {
      frameOrientation.checkReferenceFrameMatch(getParent());
      originPose.setOrientation(frameOrientation);
      this.update();
   }

   public void setPoseAndUpdate(Point2DReadOnly position, double orientation)
   {
      originPose.setPosition(position);
      originPose.setOrientation(orientation);
      this.update();
   }

   public void setPoseAndUpdate(FramePose2D pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(FramePoint2D position, FrameOrientation2D orientation)
   {
      position.changeFrame(originPose.getReferenceFrame());
      originPose.setPosition(position);

      orientation.changeFrame(originPose.getReferenceFrame());
      originPose.setOrientation(orientation);
      this.update();
   }

   public FramePose2D getPoseCopy()
   {
      return new FramePose2D(originPose);
   }

   @Override
   public String toString()
   {
      return super.toString() + ", originPose = " + originPose;
   }
}
