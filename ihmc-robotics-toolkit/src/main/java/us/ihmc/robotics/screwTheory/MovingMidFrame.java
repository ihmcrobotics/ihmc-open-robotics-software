package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FramePose;

/**
 * {@code MovingMidFrame} represents a the average of two given reference frames.
 *
 */
public class MovingMidFrame extends MovingReferenceFrame
{
   private final MovingReferenceFrame frameOne, frameTwo;
   private final FramePose pose = new FramePose();
   private final FramePose poseOne = new FramePose();
   private final FramePose poseTwo = new FramePose();

   public MovingMidFrame(String name, MovingReferenceFrame frameOne, MovingReferenceFrame frameTwo)
   {
      super(name, frameOne.getRootFrame());

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
      pose.getPose(transformToParent);
   }

   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearVelocityOne = new FrameVector3D();
   private final FrameVector3D linearVelocityTwo = new FrameVector3D();

   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularVelocityOne = new FrameVector3D();
   private final FrameVector3D angularVelocityTwo = new FrameVector3D();

   private final Quaternion difference = new Quaternion();
   private final Quaternion quaternionFuture = new Quaternion();
   private final Quaternion quaternionFutureOne = new Quaternion();
   private final Quaternion quaternionFutureTwo = new Quaternion();

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      twistRelativeToParentToPack.setToZero(this, parentFrame, this);

      Twist twistOfFrameOne = frameOne.getTwistOfFrame();
      Twist twistOfFrameTwo = frameTwo.getTwistOfFrame();

      twistOfFrameOne.getLinearPart(linearVelocityOne);
      twistOfFrameTwo.getLinearPart(linearVelocityTwo);

      linearVelocityOne.changeFrame(this);
      linearVelocityTwo.changeFrame(this);
      linearVelocity.setToZero(this);
      linearVelocity.interpolate(linearVelocityOne, linearVelocityTwo, 0.5);
      twistRelativeToParentToPack.setLinearPart(linearVelocity);

      twistOfFrameOne.getAngularPart(angularVelocityOne);
      twistOfFrameTwo.getAngularPart(angularVelocityTwo);

      computeAngularVelocityNumeric();

      twistRelativeToParentToPack.setAngularPart(angularVelocity);
   }

   /**
    * Not happy with this solution, but computing the angular velocity of the average orientation
    * appears to be rather complex.
    * <p>
    * The variables are the following:
    * <ul>
    * <li>The frame one has an orientation q<sub>1</sub> and an angular velocity &omega<sub>1</sub>.
    * <li>The frame two has an orientation q<sub>2</sub> and an angular velocity &omega<sub>2</sub>.
    * </ul>
    * This mid frame has an orientation q<sub>i</sub> located in the middle of q<sub>1</sub> and
    * q<sub>2</sub>. It is calculated as follows:<br>
    * q<sub>i</sub> = q<sub>1</sub> (q<sub>1</sub><sup>-1</sup>q<sub>2</sub>)<sup>0.5</sup>
    * </p>
    * When comparing against finite difference, it is clear that: &omega;<sub>i</sub> &ne; 0.5
    * (&omega;<sub>1</sub> + &omega;<sub>2</sub>).
    * <p>
    * <p>
    * Therefore, the time-derivative of the equation used to compute q<sub>i</sub> needs to be
    * calculated using quaternion differential calculus. However, I keep getting stuck at computing
    * the time-derivative of a quaternion raised to a power:<br>
    * 
    * <pre>
    * d q(t)<sup>&alpha;</sup>
    * ------   = ???
    *  dt
    * </pre>
    * 
    * where in our case &alpha; = 0.5.
    * </p>
    * <p>
    * So as a workaround, using the known angular velocities, q<sub>i</sub> is evaluated at
    * t+&Delta;t and the angular velocity computed from finite difference. This should behave better
    * than simply using {@code NumericalMovingReferenceFrame} which does not take into account the
    * angular velocities of the two input frames.
    * </p>
    */
   public void computeAngularVelocityNumeric()
   {
      double integrationDT = 0.00001;

      angularVelocityOne.scale(integrationDT);
      angularVelocityTwo.scale(integrationDT);

      quaternionFutureOne.set(angularVelocityOne.getVector());
      quaternionFutureTwo.set(angularVelocityTwo.getVector());

      quaternionFutureOne.preMultiply(poseOne.getOrientation());
      quaternionFutureTwo.preMultiply(poseTwo.getOrientation());

      quaternionFuture.interpolate(quaternionFutureOne, quaternionFutureTwo, 0.5);
      difference.difference(pose.getOrientation(), quaternionFuture);
      angularVelocity.setToZero(this);
      difference.get(angularVelocity.getVector());
      angularVelocity.scale(1.0 / integrationDT);
   }
}
