package us.ihmc.valkyrie.parameters;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import us.ihmc.humanoidRobotics.footstep.footstepGenerator.UIFootstepGeneratorParameters;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieCollisionBasedSelectionModel;
import us.ihmc.wholeBodyController.UIParameters;

public class ValkyrieUIParameters implements UIParameters
{
   private final ValkyriePhysicalProperties physicalProperties;
   private final ValkyrieJointMap jointMap;

   public ValkyrieUIParameters(ValkyriePhysicalProperties physicalProperties, ValkyrieJointMap jointMap)
   {
      this.physicalProperties = physicalProperties;
      this.jointMap = jointMap;
   }

   @Override
   public double getAnkleHeight()
   {
      return physicalProperties.getAnkleHeight();
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
      return (1 + 0.3) * 2 * Math.sqrt(physicalProperties.getFootForward() * physicalProperties.getFootForward()
            + 0.25 * physicalProperties.getFootWidth() * physicalProperties.getFootWidth());
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

   @Override
   public RobotCollisionModel getSelectionModel()
   {
      return new ValkyrieCollisionBasedSelectionModel(jointMap);
   }

   @Override
   public UIFootstepGeneratorParameters getUIFootstepGeneratorParameters()
   {
      return new ValkyrieUIFootstepGeneratorParameters();
   }
}
