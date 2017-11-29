package us.ihmc.humanoidRobotics.frames;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.screwTheory.MovingMidFootZUpGroundFrame;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

/**
 * {@code MovingWalkingReferenceFrame} was originally implemented in
 * {@code HumanoidReferenceFrames}.
 * <p>
 * It represents the need of having some simple reference frame moving along the walking path
 * somewhere in between in the feet and moving forward in similar way as the robot is progressing.
 * </p>
 * <p>
 * This implementation starts off a {@code MovingMidFootZUpGroundFrame} and adjust it to have the
 * same progression as the pelvis.
 * </p>
 */
public class MovingWalkingReferenceFrame extends MovingReferenceFrame
{
   private final MovingReferenceFrame pelvisFrame;
   private final MovingMidFootZUpGroundFrame midFootZUpGroundFrame;

   private final FramePoint3D pelvisPosition = new FramePoint3D();
   private final FramePose pose = new FramePose();

   public MovingWalkingReferenceFrame(String name, MovingReferenceFrame pelvisFrame, MovingMidFootZUpGroundFrame midFootZUpGroundFrame)
   {
      super(name, pelvisFrame.getRootFrame(), true);

      if (pelvisFrame == midFootZUpGroundFrame)
         throw new IllegalArgumentException("The frames have to be different.");
      pelvisFrame.verifySameRoots(midFootZUpGroundFrame);

      this.pelvisFrame = pelvisFrame;
      this.midFootZUpGroundFrame = midFootZUpGroundFrame;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      pelvisPosition.setToZero(pelvisFrame);
      pelvisPosition.changeFrame(midFootZUpGroundFrame);
      pose.setToZero(midFootZUpGroundFrame);
      pose.setX(pelvisPosition.getX());
      pose.changeFrame(parentFrame);
      pose.getPose(transformToParent);
   }

   private final FrameVector3D linearPelvisVelocity = new FrameVector3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final Twist twistOfPelvisRelativeToMidFootFrame = new Twist();
   private final Vector3D offset = new Vector3D();
   private final Vector3D linearVelocityOffset = new Vector3D();

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      twistRelativeToParentToPack.setToZero(this, parentFrame, this);

      Twist twistOfMidFootFrame = midFootZUpGroundFrame.getTwistOfFrame();
      twistOfMidFootFrame.getAngularPart(angularVelocity);
      twistOfMidFootFrame.getLinearPart(linearVelocity);

      pelvisFrame.getTwistRelativeToOther(midFootZUpGroundFrame, twistOfPelvisRelativeToMidFootFrame);

      twistOfPelvisRelativeToMidFootFrame.getLinearPart(linearPelvisVelocity);
      linearPelvisVelocity.changeFrame(this);

      angularVelocity.changeFrame(this);
      linearVelocity.changeFrame(this);
      linearVelocity.add(linearPelvisVelocity.getX(), 0.0, 0.0);

      offset.set(pelvisPosition.getX(), 0.0, 0.0);
      linearVelocityOffset.cross(angularVelocity.getVector(), offset);

      linearVelocity.add(linearVelocityOffset);

      twistRelativeToParentToPack.setAngularPart(angularVelocity);
      twistRelativeToParentToPack.setLinearPart(linearVelocity);
   }
}
