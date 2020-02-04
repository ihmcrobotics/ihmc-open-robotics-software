package us.ihmc.robotEnvironmentAwareness.slam.optimization;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public abstract class SecondStageSLAMFrameOptimizerCostFunction implements SingleQueryFunction
{
   protected final RigidBodyTransformReadOnly transformWorldToSensorPose;
   private final boolean enableYaw;
   private final RigidBodyTransform yawRotator = new RigidBodyTransform();

   public SecondStageSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, boolean enableYaw)
   {
      this.transformWorldToSensorPose = transformWorldToSensorPose;
      this.enableYaw = enableYaw;
   }

   public void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      RigidBodyTransform newSensorPose = new RigidBodyTransform();
      convertToSensorPose(input, newSensorPose);
      
      newSensorPose.normalizeRotationPart();
      newSensorPose.preMultiplyInvertOther(transformWorldToSensorPose);
      
      transformToPack.set(newSensorPose);
   }

   public void convertToSensorPose(TDoubleArrayList input, RigidBodyTransform sensorPoseToPack)
   {
      sensorPoseToPack.set(transformWorldToSensorPose);
      sensorPoseToPack.appendTranslation(input.get(0), input.get(1), input.get(2));
      if (enableYaw)
      {
         yawRotator.setIdentity();
         yawRotator.appendYawRotation(input.get(3) * 5);
         sensorPoseToPack.preMultiply(yawRotator);
      }
   }

   public void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      RigidBodyTransform newSensorPose = new RigidBodyTransform();
      convertToSensorPose(input, newSensorPose);

      transformToPack.set(newSensorPose);
      transformToPack.multiplyInvertOther(transformWorldToSensorPose);
   }
}
