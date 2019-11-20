package us.ihmc.valkyrie.parameters;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.UIParameters;

public class ValkyrieUIParameters implements UIParameters
{

   @Override
   public double getAnkleHeight()
   {
      return ValkyriePhysicalProperties.ankleHeight;
   }

   /** @inheritDoc */
   @Override
   public double getSpineYawLimit()
   {
      return Math.PI / 4.0;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchUpperLimit()
   {
      return -0.13;
   }

   /** @inheritDoc */
   @Override
   public double getSpinePitchLowerLimit()
   {
      return 0.666;
   }

   /** @inheritDoc */
   @Override
   public double getSpineRollLimit()
   {
      return Math.PI / 4.0;
   }

   /** @inheritDoc */
   @Override
   public boolean isSpinePitchReversed()
   {
      return true;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(ValkyriePhysicalProperties.footForward * ValkyriePhysicalProperties.footForward
            + 0.25 * ValkyriePhysicalProperties.footWidth * ValkyriePhysicalProperties.footWidth);
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0.8157;
   }

   /** {@inheritDoc} */
   @Override
   public double getDefaultTrajectoryTime()
   {
      return 2.0;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide robotSide)
   {
      Vector3f centerOfHandToWristTranslation = new Vector3f();
      float[] angles = new float[3];

      centerOfHandToWristTranslation = new Vector3f(0f, robotSide.negateIfLeftSide(0.015f), -0.06f);
      angles[0] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));
      angles[1] = 0.0f;
      angles[2] = (float) robotSide.negateIfLeftSide(Math.toRadians(90));

      Quaternion centerOfHandToWristRotation = new Quaternion(angles);
      return new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation);
   }
}
