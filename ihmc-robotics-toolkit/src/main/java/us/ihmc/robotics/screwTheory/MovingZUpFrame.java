package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class MovingZUpFrame extends MovingReferenceFrame
{

   private final ReferenceFrame rootFrame;
   private final MovingReferenceFrame nonZUpFrame;

   private final double[] yawPitchRoll = new double[3];

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

      // If we get the values from the built in method in the transformToParent a pitch close to
      // pi/2 will cause the values to become nan. This is because it becomes numerically hard
      // to determine yaw at that point. However, for a reference frame that behavior can cause
      // controller crashes. It is better to accept that the yaw will not be precise in that case.
      yawPitchRoll[0] = Math.atan2(transformToParent.getM10(), transformToParent.getM00());
      yawPitchRoll[1] = Math.asin(-transformToParent.getM20());
      yawPitchRoll[2] = Math.atan2(transformToParent.getM21(), transformToParent.getM22());

      transformToParent.setRotationYaw(yawPitchRoll[0]);
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
    */
   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      nonZUpFrame.getTwistOfFrame(twistRelativeToParentToPack);

      double yawDot = computeYawRate(yawPitchRoll, twistRelativeToParentToPack.getAngularPart(), true);

      twistRelativeToParentToPack.changeFrame(this);
      twistRelativeToParentToPack.changeBodyFrameNoRelativeTwist(this);
      twistRelativeToParentToPack.changeBaseFrameNoRelativeTwist(rootFrame);

      twistRelativeToParentToPack.setAngularPart(0.0, 0.0, yawDot);
   }

   /**
    * Computes the yaw angle rate of the yaw-pitch-roll representation of a orientation given the
    * angular velocity.
    * 
    * @param yawPitchRoll the Euler angles describing a 3D rotation. Not modified.
    * @param angularVelocity the angular velocity. Not modified.
    * @param isVelocityInLocalCoordinates whether the angular velocity is expressed in the
    *           coordinates described by the yaw-pitch-roll angles or in the base coordinates of the
    *           yaw-pitch-roll angles.
    * @return the value of the rate of change of the yaw angle.
    */
   public static double computeYawRate(double[] yawPitchRoll, Vector3DReadOnly angularVelocity, boolean isVelocityInLocalCoordinates)
   {
      double wx = angularVelocity.getX();
      double wy = angularVelocity.getY();
      double wz = angularVelocity.getZ();

      double yaw = yawPitchRoll[0];
      double pitch = yawPitchRoll[1];
      double roll = yawPitchRoll[2];

      if (isVelocityInLocalCoordinates)
         return (Math.sin(roll) * wy + Math.cos(roll) * wz) / Math.cos(pitch);
      else
         return wz + Math.tan(pitch) * (Math.sin(yaw) * wy + Math.cos(yaw) * wx);
   }
}
