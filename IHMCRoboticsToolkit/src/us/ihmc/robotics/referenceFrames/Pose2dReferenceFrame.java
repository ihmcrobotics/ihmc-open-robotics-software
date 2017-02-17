package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FrameOrientation2d;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose2d;

public class Pose2dReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 1559362743221742457L;
   private final FramePose2d originPose;

   public Pose2dReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);

      originPose = new FramePose2d(parentFrame);
   }

   public Pose2dReferenceFrame(String frameName, FramePose2d pose)
   {
      this(frameName, pose.getReferenceFrame());
      setPoseAndUpdate(pose);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      originPose.getPose(transformToParent);
   }

   public void setPositionAndUpdate(FramePoint2d framePoint)
   {
      framePoint.checkReferenceFrameMatch(parentFrame);
      originPose.setPosition(framePoint);
      this.update();
   }

   public void setOrientationAndUpdate(FrameOrientation2d frameOrientation)
   {
      frameOrientation.checkReferenceFrameMatch(parentFrame);
      originPose.setOrientation(frameOrientation);
      this.update();
   }

   public void setPoseAndUpdate(FramePose2d pose)
   {
      originPose.set(pose);
      this.update();
   }

   public void setPoseAndUpdate(FramePoint2d position, FrameOrientation2d orientation)
   {
      position.changeFrame(originPose.getReferenceFrame());
      originPose.setPosition(position);

      orientation.changeFrame(originPose.getReferenceFrame());
      originPose.setOrientation(orientation);
      this.update();
   }

   public FramePose2d getPoseCopy()
   {
      return new FramePose2d(originPose);
   }

   @Override
   public String toString()
   {
      return super.toString() + ", originPose = " + originPose;
   }
}
