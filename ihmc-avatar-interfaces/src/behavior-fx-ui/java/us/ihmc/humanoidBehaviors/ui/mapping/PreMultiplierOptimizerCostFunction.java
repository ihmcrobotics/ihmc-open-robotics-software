package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.numericalMethods.SingleQueryFunction;

public class PreMultiplierOptimizerCostFunction implements SingleQueryFunction
{
   private PlanarRegionsList map;
   private List<Plane3D> planes;
   private static final double angleScaler = 1.0;
   private static final double angleWeight = 0.5;
   private static final double snanppingWeight = 0.05;
   private RigidBodyTransformReadOnly transformWorldToSensorPose;

   public PreMultiplierOptimizerCostFunction(PlanarRegionsList map, List<Plane3D> planes, RigidBodyTransformReadOnly transformWorldToSensorPose)
   {
      this.map = map;
      this.planes = planes;
      this.transformWorldToSensorPose = transformWorldToSensorPose;
   }
   
   public void convertToSensorPoseMultiplier(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      transformToPack.setTranslation(input.get(0), input.get(1), input.get(2));
      transformToPack.setRotationYawPitchRoll(input.get(3) / angleScaler, input.get(4) / angleScaler, input.get(5) / angleScaler);
   }

   public void convertToPointCloudTransformer(TDoubleArrayList input, RigidBodyTransform transformToPack)
   {
      RigidBodyTransform preMultiplier = new RigidBodyTransform();
      convertToSensorPoseMultiplier(input, preMultiplier);
      
      transformToPack.set(transformWorldToSensorPose);
      transformToPack.multiply(preMultiplier);
      transformToPack.multiplyInvertOther(transformWorldToSensorPose);
   }

   @Override
   public double getQuery(TDoubleArrayList values)
   {
      /**
       * values : dx, dy, dz, du, dv, dw
       */
      RigidBodyTransform transformer = new RigidBodyTransform();
      convertToPointCloudTransformer(values, transformer);

      List<Plane3D> convertedPlanes = new ArrayList<>();
      for (int i = 0; i < planes.size(); i++)
      {
         Vector3D convertedNormal = planes.get(i).getNormalCopy();
         Point3D convertedCenter = planes.get(i).getPointCopy();

         transformer.transform(convertedNormal);
         transformer.transform(convertedCenter);
         convertedPlanes.add(new Plane3D(convertedCenter, convertedNormal));
      }

      double cost = 0;
      int numberOfPlanarRegions = map.getNumberOfPlanarRegions();
      for (int i = 0; i < convertedPlanes.size(); i++)
      {
         double minimumDistance = Double.MAX_VALUE;
         int index = -1;
         for (int j = 0; j < numberOfPlanarRegions; j++)
         {
            PlanarRegion planarRegion = map.getPlanarRegion(j);
            Plane3D plane = planarRegion.getPlane();
            double distance = plane.distance(convertedPlanes.get(i).getPoint());
            if (distance < minimumDistance)
            {
               minimumDistance = distance;
               index = j;
            }
         }

         PlanarRegion planarRegion = map.getPlanarRegion(index);
         Plane3D plane = planarRegion.getPlane();
         double angleDistance = 1 - Math.abs(plane.getNormal().dot(convertedPlanes.get(i).getNormal()));
         double centerDistance = 0;
         if (!planarRegion.isPointInsideByProjectionOntoXYPlane(convertedPlanes.get(i).getPoint()))
            centerDistance = plane.getPoint().distance(convertedPlanes.get(i).getPoint());

         double distanceCost = minimumDistance + angleWeight * angleDistance + snanppingWeight * centerDistance;
         {
            cost = cost + distanceCost;
         }
      }

      return cost;
   }

}
