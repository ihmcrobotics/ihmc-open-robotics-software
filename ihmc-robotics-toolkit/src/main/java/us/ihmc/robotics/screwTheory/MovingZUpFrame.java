package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;

public class MovingZUpFrame extends MovingReferenceFrame
{
   private final ReferenceFrame rootFrame;
   private final MovingReferenceFrame nonZUpFrame;

   private double sinRoll = 0.0;
   private double cosRoll = 1.0;
   private double cosPitch = 1.0;

   public MovingZUpFrame(MovingReferenceFrame nonZUpFrame, String name)
   {
      super(name, nonZUpFrame.getRootFrame(), true);

      this.rootFrame = nonZUpFrame.getRootFrame();
      this.nonZUpFrame = nonZUpFrame;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      nonZUpFrame.getTransformToDesiredFrame(transformToParent, rootFrame);

      // Compute the yaw rotation matrix while avoiding the computation of the actual yaw-pitch-roll angles.
      double sinPitch = -transformToParent.getM20();
      cosPitch = Math.sqrt(1.0 - sinPitch * sinPitch);
      cosRoll = transformToParent.getM22() / cosPitch;
      sinRoll = transformToParent.getM21() / cosPitch;
      double cosYaw = transformToParent.getM00() / cosPitch;
      double sinYaw = transformToParent.getM10() / cosPitch;

      transformToParent.setRotation(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);
   }

   /**
    * As this frame is rotating according to the yaw part of the {@code nonZUpFrame} yaw-pitch-roll
    * angles, this frame angular velocity is around z only with a magnitude equal to the derivative
    * of the yaw angle.
    * <p>
    * To compute the yaw rate, we need to transform each of the three yaw-pitch-roll velocities into
    * the {@code nonZUpFrame} to obtain the corresponding angular velocity in terms of d(yaw)/dt,
    * d(pitch)/dt, and d(roll)/dt:
    * 
    * <pre>
    *     /            d(roll)/dt - sin(pitch) * d(yaw)/dt                \
    * &omega; = |  cos(roll) * d(pitch)/dt + cos(pitch) * sin(roll) * d(yaw)/dt |
    *     \ -sin(roll) * d(pitch)/dt + cos(pitch) * cos(roll) * d(yaw)/dt /
    * </pre>
    * 
    * where &omega; is the angular velocity of the nonZUpFrame. The equality gives three equations
    * for the unknowns that are the rate of the yaw, pitch, and roll angles of the
    * {@code nonZUpFrame}, from which the yaw rate can be found:<br>
    * d(yaw)/dt = (sin(roll) * &omega;<sub>y</sub> + cos(roll) * &omega;<sub>z</sub>) /
    * cos(pitch)<br>
    * which is the z component of this frame angular velocity.
    * </p>
    * 
    * @see RotationTools#computeYawRate(double[], us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly, boolean)
    */
   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      nonZUpFrame.getTwistOfFrame(twistRelativeToParentToPack);

      // We avoid computing yaw-pitch-roll to reduce computation cost.
      double wy = twistRelativeToParentToPack.getAngularPartY();
      double wz = twistRelativeToParentToPack.getAngularPartZ();
      double yawDot = (sinRoll * wy + cosRoll * wz) / cosPitch;

      twistRelativeToParentToPack.changeFrame(this);
      twistRelativeToParentToPack.changeBodyFrameNoRelativeTwist(this);
      twistRelativeToParentToPack.changeBaseFrameNoRelativeTwist(rootFrame);
      twistRelativeToParentToPack.setAngularPart(0.0, 0.0, yawDot);
   }
}
