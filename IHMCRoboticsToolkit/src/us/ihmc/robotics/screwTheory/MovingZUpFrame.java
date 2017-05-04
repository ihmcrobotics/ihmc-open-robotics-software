package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class MovingZUpFrame extends MovingReferenceFrame
{
   private static final long serialVersionUID = 2228394703194703367L;

   private final ReferenceFrame rootFrame;
   private final MovingReferenceFrame nonZUpFrame;

   private final double[] yawPitchRoll = new double[3];

   public MovingZUpFrame(MovingReferenceFrame nonZUpFrame, String name, YoVariableRegistry registry)
   {
      super(name, nonZUpFrame.getRootFrame(), true);
      this.rootFrame = nonZUpFrame.getRootFrame();
      this.nonZUpFrame = nonZUpFrame;
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      nonZUpFrame.getTransformToDesiredFrame(transformToParent, rootFrame);
      transformToParent.getRotationYawPitchRoll(yawPitchRoll);
      transformToParent.setRotationYaw(yawPitchRoll[0]);
   }

   /**
    * As the frame is rotating according to the yaw part of the nonZUpFrame yaw-pitch-roll, this
    * frame angular velocity is around z only with a magnitude equal to the derivative of the yaw
    * angle.
    * <p>
    * To compute the yaw rate, we need to transform each of the three yaw-pitch-roll velocities into
    * the nonZUpFrame to obtain the corresponding angular velocity in terms of d(yaw)/dt,
    * d(pitch)/dt, and d(roll)/dt:
    * 
    * <pre>
    *     /            d(roll)/dt - sin(pitch) * d(yaw)/dt                \
    * &omega; = |  cos(roll) * d(pitch)/dt + cos(pitch) * sin(roll) * d(yaw)/dt |
    *     \ -sin(roll) * d(pitch)/dt + cos(pitch) * cos(roll) * d(yaw)/dt /
    * </pre>
    * 
    * where &omega; is the angular velocity of the nonZUpFrame. The equality gives three equations
    * for the unknowns that are the rate of the yaw, pitch, and roll, from which the yaw rate can be
    * found:<br>
    * d(yaw)/dt = (sin(roll) * &omega;<sub>y</sub> + cos(roll) * &omega;<sub>z</sub>) / cos(pitch)<br>
    * which is the z component of this frame angular velocity.
    * </p>
    */
   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      nonZUpFrame.getTwistOfFrame(twistRelativeToParentToPack);

      double wy = twistRelativeToParentToPack.getAngularPartY();
      double wz = twistRelativeToParentToPack.getAngularPartZ();

      double roll = yawPitchRoll[2];
      double pitch = yawPitchRoll[1];

      double yawDot = Math.sin(roll) * wy + Math.cos(roll) * wz;
      yawDot /= Math.cos(pitch);

      twistRelativeToParentToPack.changeFrame(this);
      twistRelativeToParentToPack.changeBodyFrameNoRelativeTwist(this);
      twistRelativeToParentToPack.changeBaseFrameNoRelativeTwist(rootFrame);

      twistRelativeToParentToPack.setAngularPart(0.0, 0.0, yawDot);
   }
}
