package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.octreeBasedSurfaceElement;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;

public class OctreeSLAMFrameOptimizerCostFunction extends SLAMFrameOptimizerCostFunction
{
   private List<IhmcSurfaceElement> surfaceElements;
   private static final double POSITION_WEIGHT = 5.0;
   private static final double ANGLE_WEIGHT = 5.0;
   private static final double SNAPPING_PARALLEL_WEIGHT = 5.0;

   private static final double LEAST_SQUARE_WEIGHT = 0.1;

   private boolean assumeFlatGround = true;

   public OctreeSLAMFrameOptimizerCostFunction(List<IhmcSurfaceElement> surfaceElements, RigidBodyTransformReadOnly transformWorldToSensorPose)
   {
      super(transformWorldToSensorPose);
      this.surfaceElements = surfaceElements;

      int firstPlanarRegionId = surfaceElements.get(0).getMergeablePlanarRegionId();
      for (IhmcSurfaceElement surfaceElement : surfaceElements)
      {
         if (firstPlanarRegionId != surfaceElement.getMergeablePlanarRegionId())
            assumeFlatGround = false;
      }
   }

   @Override
   public double getQuery(TDoubleArrayList values)
   {
      /**
       * values are difference in 6 dimensions : dx, dy, dz, du, dv, dw
       */
      RigidBodyTransform transformer = new RigidBodyTransform();
      convertToPointCloudTransformer(values, transformer);
      //convertToSensorPoseMultiplier(values, transformer);

      List<IhmcSurfaceElement> convertedElements = new ArrayList<>();
      for (int i = 0; i < surfaceElements.size(); i++)
      {
         Vector3D convertedNormal = surfaceElements.get(i).getPlane().getNormalCopy();
         Point3D convertedCenter = surfaceElements.get(i).getPlane().getPointCopy();

         transformer.transform(convertedNormal);
         transformer.transform(convertedCenter);
         IhmcSurfaceElement convertedElement = new IhmcSurfaceElement(surfaceElements.get(i));
         convertedElement.transform(transformer);
         convertedElements.add(convertedElement);
      }

      double cost = 0;
      int cnt = 0;
      for (int i = 0; i < convertedElements.size(); i++)
      {
         IhmcSurfaceElement convertedElement = convertedElements.get(i);
         double distance = convertedElement.getDistance(POSITION_WEIGHT, ANGLE_WEIGHT);

         double squareOfInput = 0.0;
         for (double value : values.toArray())
         {
            squareOfInput = squareOfInput + value * value;
         }

         double distanceCost = distance + LEAST_SQUARE_WEIGHT * squareOfInput;
         {
            cost = cost + distanceCost;
         }
         if (convertedElement.isInPlanarRegion())
            cnt++;
      }

      double snappingScore;
      if (assumeFlatGround)
         snappingScore = 0.0;
      else
         snappingScore = 1 - (double) cnt / convertedElements.size();
      cost = cost + snappingScore * SNAPPING_PARALLEL_WEIGHT;

      return cost;
   }
}
