package us.ihmc.humanoidBehaviors.ui.mapping;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public abstract class SLAMFrameOptimizerCostFunction implements SingleQueryFunction
{
   private static final double ANGLE_SCALER = 0.2; //TODO:
   protected final RigidBodyTransformReadOnly transformWorldToSensorPose;
   public Point3D[] staticPoints = new Point3D[200];
   public SLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose)
   {
      this.transformWorldToSensorPose = transformWorldToSensorPose;
      for (int i = 0; i < staticPoints.length; i++)
         staticPoints[i] = new Point3D();
   }

   /**
    * newSensorPose = originalSensorPose * transformToPack;
    */
   protected void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      transformToPack.setTranslation(input.get(0), input.get(1), input.get(2));
      transformToPack.setRotationYawPitchRoll(input.get(5) / ANGLE_SCALER, input.get(4) / ANGLE_SCALER, input.get(3) / ANGLE_SCALER); // TODO: improve this.
   }

   protected void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      convertToSensorPoseMultiplier(input, preMultiplier);

      transformToPack.set(transformWorldToSensorPose);
      transformToPack.multiply(preMultiplier);
      transformToPack.multiplyInvertOther(transformWorldToSensorPose);
   }
}
