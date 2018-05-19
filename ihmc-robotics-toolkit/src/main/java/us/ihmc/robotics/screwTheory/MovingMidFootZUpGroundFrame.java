package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class MovingMidFootZUpGroundFrame extends MovingReferenceFrame
{
   private final MovingZUpFrame frameOne, frameTwo;
   private final FramePose3D pose = new FramePose3D();
   private final FramePose3D poseOne = new FramePose3D();
   private final FramePose3D poseTwo = new FramePose3D();

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearVelocityOne = new FrameVector3D();
   private final FrameVector3D linearVelocityTwo = new FrameVector3D();

   public MovingMidFootZUpGroundFrame(String name, MovingZUpFrame frameOne, MovingZUpFrame frameTwo)
   {
      super(name, frameOne.getRootFrame(), true);

      if (frameOne == frameTwo)
         throw new IllegalArgumentException("The frames have to be different.");
      frameOne.verifySameRoots(frameTwo);

      this.frameOne = frameOne;
      this.frameTwo = frameTwo;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      poseOne.setToZero(frameOne);
      poseTwo.setToZero(frameTwo);

      poseOne.changeFrame(parentFrame);
      poseTwo.changeFrame(parentFrame);

      pose.setToZero(parentFrame);
      pose.interpolate(poseOne, poseTwo, 0.5);
      pose.setZ(Math.min(poseOne.getZ(), poseTwo.getZ()));
      pose.get(transformToParent);
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      Twist twistOfFrameOne = frameOne.getTwistOfFrame();
      Twist twistOfFrameTwo = frameTwo.getTwistOfFrame();

      twistOfFrameOne.getLinearPart(linearVelocityOne);
      twistOfFrameTwo.getLinearPart(linearVelocityTwo);

      linearVelocityOne.changeFrame(this);
      linearVelocityTwo.changeFrame(this);
      linearVelocity.setToZero(this);
      linearVelocity.interpolate(linearVelocityOne, linearVelocityTwo, 0.5);
      twistRelativeToParentToPack.setToZero(this, parentFrame, this);
      twistRelativeToParentToPack.setLinearPart(linearVelocity);

      if (poseOne.getZ() < poseTwo.getZ())
         twistRelativeToParentToPack.setLinearPartZ(linearVelocityOne.getZ());
      else
         twistRelativeToParentToPack.setLinearPartZ(linearVelocityTwo.getZ());

      double wz = 0.5 * (twistOfFrameOne.getAngularPartZ() + twistOfFrameTwo.getAngularPartZ());
      twistRelativeToParentToPack.setAngularPartZ(wz);
   }
}
