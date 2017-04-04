package us.ihmc.robotics.referenceFrames;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.RotationTools;

public class MidFrameZUpAverageYawFrame extends ReferenceFrame
{
   private static final long serialVersionUID = -8299445128076938283L;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ReferenceFrame frameOne, frameTwo;
   private final FramePose framePose = new FramePose();
   private final FramePose poseOne = new FramePose();
   private final FramePose poseTwo = new FramePose();

   public MidFrameZUpAverageYawFrame(String name, ReferenceFrame frameOne, ReferenceFrame frameTwo)
   {
      super(name, worldFrame, false, false, true);
      this.frameOne = frameOne;
      this.frameTwo = frameTwo;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      poseOne.setToZero(frameOne);
      poseTwo.setToZero(frameTwo);

      poseOne.changeFrame(worldFrame);
      poseTwo.changeFrame(worldFrame);

      framePose.interpolate(poseOne, poseTwo, 0.5);
      framePose.setZ(Math.min(poseOne.getZ(), poseTwo.getZ()));
      framePose.getPose(transformToParent);
      RotationTools.removePitchAndRollFromTransform(transformToParent);
   }
}
