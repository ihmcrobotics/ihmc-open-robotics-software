package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
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

   public MovingZUpFrame(MovingReferenceFrame nonZUpFrame, ReferenceFrame parentFrame, String name)
   {
      super(name, parentFrame, true);

      this.rootFrame = parentFrame;
      this.nonZUpFrame = nonZUpFrame;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      nonZUpFrame.getTransformToDesiredFrame(transformToParent, rootFrame);

      // Compute the yaw rotation matrix while avoiding the computation of the actual yaw-pitch-roll angles.
      double sinPitch = -transformToParent.getM20();
      cosPitch = Math.sqrt(1.0 - sinPitch * sinPitch);

      if (EuclidCoreTools.isZero(cosPitch, 1.0e-12))
      { // pitch = Pi/2 best thing to do is to set the rotation to identity.
         cosRoll = 1.0;
         sinRoll = 0.0;
         transformToParent.getRotation().setIdentity();
      }
      else
      {
         cosRoll = transformToParent.getM22() / cosPitch;
         sinRoll = transformToParent.getM21() / cosPitch;
         double invNormRoll = 1.0 / EuclidCoreTools.norm(cosRoll, sinRoll);
         cosRoll *= invNormRoll;
         sinRoll *= invNormRoll;

         double cosYaw = transformToParent.getM00() / cosPitch;
         double sinYaw = transformToParent.getM10() / cosPitch;
         double invNormYaw = 1.0 / EuclidCoreTools.norm(cosYaw, sinYaw);
         cosYaw *= invNormYaw;
         sinYaw *= invNormYaw;

         transformToParent.getRotation().setUnsafe(cosYaw, -sinYaw, 0.0, sinYaw, cosYaw, 0.0, 0.0, 0.0, 1.0);
      }
   }

   /**
    * As this frame is rotating according to the yaw part of the {@code nonZUpFrame} yaw-pitch-roll
    * angles, this frame angular velocity is around z only with a magnitude equal to the derivative of
    * the yaw angle.
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
    * where &omega; is the angular velocity of the nonZUpFrame. The equality gives three equations for
    * the unknowns that are the rate of the yaw, pitch, and roll angles of the {@code nonZUpFrame},
    * from which the yaw rate can be found:<br>
    * d(yaw)/dt = (sin(roll) * &omega;<sub>y</sub> + cos(roll) * &omega;<sub>z</sub>) / cos(pitch)<br>
    * which is the z component of this frame angular velocity.
    * </p>
    * 
    * @see RotationTools#computeYawRate(double[], us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly,
    *      boolean)
    */
   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      nonZUpFrame.getTwistOfFrame(twistRelativeToParentToPack);

      // We avoid computing yaw-pitch-roll to reduce computation cost.
      double yawDot;
      if (EuclidCoreTools.isZero(cosPitch, 1.0e-12))
      {
         yawDot = 0.0;
      }
      else
      {
         double wy = twistRelativeToParentToPack.getAngularPartY();
         double wz = twistRelativeToParentToPack.getAngularPartZ();
         yawDot = (sinRoll * wy + cosRoll * wz) / cosPitch;
      }

      twistRelativeToParentToPack.changeFrame(this);
      twistRelativeToParentToPack.setBodyFrame(this);
      twistRelativeToParentToPack.setBaseFrame(rootFrame);
      twistRelativeToParentToPack.getAngularPart().set(0.0, 0.0, yawDot);
   }
}
