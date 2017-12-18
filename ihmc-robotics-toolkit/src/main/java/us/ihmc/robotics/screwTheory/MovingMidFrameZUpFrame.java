package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.SideDependentList;

public class MovingMidFrameZUpFrame extends MovingReferenceFrame
{
   private final MovingReferenceFrame frameOne;
   private final MovingReferenceFrame frameTwo;

   private final FramePoint3D originOne = new FramePoint3D();
   private final FramePoint3D originTwo = new FramePoint3D();
   private final FrameVector3D vectorBetweenFrames = new FrameVector3D();
   private final Vector2D xAxis = new Vector2D();

   private final Vector3D translation = new Vector3D();

   public MovingMidFrameZUpFrame(String name, MovingReferenceFrame frameOne, MovingReferenceFrame frameTwo)
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
      originOne.setToZero(frameOne);
      originOne.changeFrame(parentFrame);

      originTwo.setToZero(frameTwo);
      originTwo.changeFrame(parentFrame);

      // Place this frame between the two frames.
      translation.interpolate(originOne.getPoint(), originTwo.getPoint(), 0.5);
      transformToParent.setTranslation(translation);

      vectorBetweenFrames.setToZero(parentFrame);
      vectorBetweenFrames.sub(originTwo, originOne);

      xAxis.set(-vectorBetweenFrames.getY(), vectorBetweenFrames.getX());

      if (xAxis.lengthSquared() < 1.0e-7)
         return;

      xAxis.normalize();
      transformToParent.setRotationYaw(Math.atan2(xAxis.getY(), xAxis.getX()));
   }

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearVelocityOne = new FrameVector3D();
   private final FrameVector3D linearVelocityTwo = new FrameVector3D();

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

      linearVelocityOne.sub(linearVelocity);
      linearVelocityTwo.sub(linearVelocity);

      originOne.setToZero(frameOne);
      originOne.changeFrame(this);

      originTwo.setToZero(frameTwo);
      originTwo.changeFrame(this);

      double distanceFromOne = EuclidCoreTools.norm(originOne.getX(), originOne.getY());
      double distanceFromTwo = EuclidCoreTools.norm(originTwo.getX(), originTwo.getY());

      double omegaZFromOne = linearVelocityOne.getX() / distanceFromOne;
      omegaZFromOne = Math.copySign(omegaZFromOne, originOne.getY());

      double omegaZFromTwo = linearVelocityTwo.getX() / distanceFromTwo;
      omegaZFromTwo = -Math.copySign(omegaZFromTwo, originTwo.getY());

      twistRelativeToParentToPack.setAngularPartZ(0.5 * (omegaZFromOne + omegaZFromTwo));
      twistRelativeToParentToPack.setAngularPartZ(omegaZFromTwo);
   }
}
