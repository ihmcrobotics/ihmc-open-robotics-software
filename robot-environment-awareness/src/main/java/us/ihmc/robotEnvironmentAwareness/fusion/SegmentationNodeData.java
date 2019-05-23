package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class SegmentationNodeData
{
   private final TIntArrayList labels = new TIntArrayList();

   private final Vector3D normal = new Vector3D();
   private final Point3D center = new Point3D();

   private double weight = 0.0;

   private final List<Point3D> pointsInSegment = new ArrayList<>();

   public SegmentationNodeData(FusionDataSegment seedImageSegment)
   {
      labels.add(seedImageSegment.getImageSegmentLabel());

      normal.set(seedImageSegment.getNormal());
      center.set(seedImageSegment.getCenter());

      pointsInSegment.addAll(seedImageSegment.getPoints());
   }

   void merge(FusionDataSegment fusionDataSegment)
   {
      labels.add(fusionDataSegment.getImageSegmentLabel());

      double otherWeight = fusionDataSegment.getWeight();
      double totalWeight = weight + otherWeight;
      normal.setX((normal.getX() * weight + fusionDataSegment.getNormal().getX() * otherWeight) / totalWeight);
      normal.setY((normal.getY() * weight + fusionDataSegment.getNormal().getY() * otherWeight) / totalWeight);
      normal.setZ((normal.getZ() * weight + fusionDataSegment.getNormal().getZ() * otherWeight) / totalWeight);

      center.setX((center.getX() * weight + fusionDataSegment.getCenter().getX() * otherWeight) / totalWeight);
      center.setY((center.getY() * weight + fusionDataSegment.getCenter().getY() * otherWeight) / totalWeight);
      center.setZ((center.getZ() * weight + fusionDataSegment.getCenter().getZ() * otherWeight) / totalWeight);

      weight = totalWeight;
      pointsInSegment.addAll(fusionDataSegment.getPoints());
   }

   void extend(FusionDataSegment fusionDataSegment, double threshold, boolean updateNodeData, double extendingThreshold)
   {
      for (Point3D point : fusionDataSegment.getPoints())
      {
         double distance = distancePlaneToPoint(normal, center, point);
         if (distance < threshold)
         {
            for (Point3D pointInSegment : pointsInSegment)
            {
               if (pointInSegment.distance(point) < extendingThreshold)
               {
                  pointsInSegment.add(point);
                  break;
               }
            }
         }
      }

      if (updateNodeData)
      {
         PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();

         pca.clear();
         pointsInSegment.stream().forEach(point -> pca.addPoint(point.getX(), point.getY(), point.getZ()));
         pca.compute();

         pca.getMean(center);
         pca.getThirdVector(normal);

         if (normal.getZ() < 0.0)
            normal.negate();
      }
   }

   public TIntArrayList getLabels()
   {
      return labels;
   }

   public Vector3D getNormal()
   {
      return normal;
   }

   public Point3D getCenter()
   {
      return center;
   }

   public List<Point3D> getPointsInSegment()
   {
      return pointsInSegment;
   }

   boolean isCoplanar(FusionDataSegment fusionDataSegment, double threshold)
   {
      double distanceFromSegment = distancePlaneToPoint(fusionDataSegment.getNormal(), fusionDataSegment.getCenter(), center);
      double distanceToSegment = distancePlaneToPoint(normal, center, fusionDataSegment.getCenter());

      if (Math.abs(distanceFromSegment) < threshold && Math.abs(distanceToSegment) < threshold)
         return true;
      else
         return false;
   }

   boolean isParallel(FusionDataSegment fusionDataSegment, double threshold)
   {
      if (Math.abs(fusionDataSegment.getNormal().dot(normal)) > threshold)
         return true;
      else
         return false;
   }

   public static double distancePlaneToPoint(Vector3D normalVector, Point3D center, Point3D point)
   {
      Vector3D centerVector = new Vector3D(center);
      double constantD = -normalVector.dot(centerVector);

      Vector3D pointVector = new Vector3D(point);
      return Math.abs(normalVector.dot(pointVector) + constantD) / Math.sqrt(normalVector.lengthSquared());
   }
}
