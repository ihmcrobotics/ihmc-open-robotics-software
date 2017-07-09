package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;

public class MovingMidFootZUpGroundFrame extends MovingReferenceFrame
{
   private static final long serialVersionUID = -4290437020026636187L;
   private final MovingZUpFrame frameOne, frameTwo;
   private final FramePose pose = new FramePose();
   private final FramePose poseOne = new FramePose();
   private final FramePose poseTwo = new FramePose();

   private final FrameVector linearVelocity = new FrameVector();
   private final FrameVector linearVelocityOne = new FrameVector();
   private final FrameVector linearVelocityTwo = new FrameVector();

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
      pose.getPose(transformToParent);
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
