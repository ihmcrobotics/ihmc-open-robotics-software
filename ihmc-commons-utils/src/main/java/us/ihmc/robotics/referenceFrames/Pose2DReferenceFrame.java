package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class Pose2DReferenceFrame extends ReferenceFrame
{
   private final FramePose2D originPose;

   public Pose2DReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      originPose = new FramePose2D(parentFrame);
   }

   public Pose2DReferenceFrame(String frameName, FramePose2D pose)
   {
      this(frameName, pose.getReferenceFrame());
      setPoseAndUpdate(pose);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      originPose.get(transformToParent);
   }

   public void setPositionAndUpdate(FramePoint2DReadOnly framePoint)
   {
      framePoint.checkReferenceFrameMatch(getParent());
      originPose.getPosition().set(framePoint);
      this.update();
   }

   public void setOrientationAndUpdate(FrameOrientation2DReadOnly frameOrientation)
   {
      frameOrientation.checkReferenceFrameMatch(getParent());
      originPose.getOrientation().set(frameOrientation);
      this.update();
   }

   public void setPoseAndUpdate(Point2DReadOnly position, double orientation)
   {
      originPose.getPosition().set(position);
      originPose.getOrientation().setYaw(orientation);
      this.update();
   }

   public void setPoseAndUpdate(FramePose2DReadOnly pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(FramePoint2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      originPose.setMatchingFrame(position, orientation);

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
