package us.ihmc.robotEnvironmentAwareness.slam.optimization;

import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.robotEnvironmentAwareness.slam.IhmcSurfaceElement;

public class FristStageSLAMFrameOptimizerCostFunction extends SLAMFrameOptimizerCostFunction
{
   private final List<IhmcSurfaceElement> surfaceElements;
   private static final double POSITION_WEIGHT = 0.0;
   private static final double ANGLE_WEIGHT = 5.0;

   private static final double LEAST_SQUARE_WEIGHT = 0.0;

   public FristStageSLAMFrameOptimizerCostFunction(RigidBodyTransformReadOnly transformWorldToSensorPose, List<IhmcSurfaceElement> surfaceElements)
   {
      super(transformWorldToSensorPose);
      this.surfaceElements = surfaceElements;
   }

   @Override
   public double getQuery(TDoubleArrayList values)
   {
      /**
       * values are difference in 6 dimensions : dx, dy, dz, du, dv, dw
       */
      RigidBodyTransform transformer = new RigidBodyTransform();
      convertToPointCloudTransformer(values, transformer);

      double cost = 0;
      for (int i = 0; i < surfaceElements.size(); i++)
      {
         double distance = surfaceElements.get(i).getDistance(transformer, POSITION_WEIGHT, ANGLE_WEIGHT);

         double squareOfInput = 0.0;
         for (double value : values.toArray())
         {
            squareOfInput = squareOfInput + value * value;
         }

         double distanceCost = distance + LEAST_SQUARE_WEIGHT * squareOfInput;
         {
            cost = cost + distanceCost;
         }
      }

      return cost;
   }

   /**
    * newSensorPose = originalSensorPose * transformToPack;
    */
   @Override
   public void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      transformToPack.setIdentity();
      transformToPack.setRotationYawPitchRoll(input.get(0) / ANGLE_SCALER, input.get(1) / ANGLE_SCALER, input.get(2) / ANGLE_SCALER);
   }

   @Override
   public void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      RigidBodyTransform multiplier = new RigidBodyTransform();
      convertToSensorPoseMultiplier(input, multiplier);

      transformToPack.set(transformWorldToSensorPose);
      
      transformToPack.multiply(multiplier);
      transformToPack.normalizeRotationPart();
      
      transformToPack.multiplyInvertOther(transformWorldToSensorPose);
   }

}
