package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public abstract class SLAMFrameOptimizerCostFunction implements SingleQueryFunction
{
   private static final double ANGLE_SCALER = 0.1;
   protected final RigidBodyTransformReadOnly transformWorldToSensorPose;

   public SLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose)
   {
      this.transformWorldToSensorPose = transformWorldToSensorPose;
   }

   /**
    * newSensorPose = originalSensorPose * transformToPack;
    */
   public void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      transformToPack.setTranslation(input.get(0), input.get(1), input.get(2));
      if(IhmcSLAM.ENABLE_ORIENTATION_CORRECTION)
         transformToPack.setRotationYawPitchRoll(input.get(5) / ANGLE_SCALER, input.get(4) / ANGLE_SCALER, input.get(3) / ANGLE_SCALER);
   }

   public void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      convertToSensorPoseMultiplier(input, preMultiplier);

      transformToPack.set(transformWorldToSensorPose);
      transformToPack.multiply(preMultiplier);
      transformToPack.multiplyInvertOther(transformWorldToSensorPose);
   }
}
