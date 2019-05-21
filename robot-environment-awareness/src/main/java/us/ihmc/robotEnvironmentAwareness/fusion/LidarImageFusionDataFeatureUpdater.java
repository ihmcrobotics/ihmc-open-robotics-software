package us.ihmc.robotEnvironmentAwareness.fusion;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class LidarImageFusionDataFeatureUpdater
{
   private static final double sparceThreshold = 0.01;

   private static final double proximityThreshold = 0.03;
   private static final double planarityThresholdAngle = 30.0;
   private static final double planarityThreshold = Math.cos(Math.PI / 180 * planarityThresholdAngle);

   private static final boolean updateNodeDataWithExtendedData = false;
   private static final double extendingPlaneDistanceThreshold = 0.01;
   private static final double extendingDistanceThreshold = 0.03;
   private static final int numberOfNeighborsToExtend = 50;

   private static final int maximumNumberOfTrialsToFindUnIdLabel = 100;

   private final LidarImageFusionRawData data;
   private final int numberOfLabels;
   private final List<SegmentationNodeData> segments = new ArrayList<SegmentationNodeData>();

   private final Random random = new Random(0612L);

   public LidarImageFusionDataFeatureUpdater(LidarImageFusionRawData lidarImageFusionData)
   {
      System.out.println(planarityThreshold);
      data = lidarImageFusionData;
      numberOfLabels = data.getNumberOfLabels();
   }

   public List<Point3D> getPointsOnSegment(int segmentId)
   {
      return segments.get(segmentId).pointsInSegment;
   }

   public Point3D getSegmentCenter(int segmentId)
   {
      return segments.get(segmentId).center;
   }

   public Vector3D getSegmentNormal(int segmentId)
   {
      return segments.get(segmentId).normal;
   }

   public void initialize()
   {
      segments.clear();
   }

   public boolean iterateSegmenataionPropagation(int segmentId)
   {
      int nonIDLabel = selectRandomNonIdentifiedLabel();
      LogTools.info("randomSeedLabel " + nonIDLabel);

      if (nonIDLabel == -1)
         return false;
      else
         segments.add(createSegmentNodeData(nonIDLabel, segmentId));

      return true;
   }

   public void addSegmentNodeData(int seedLabel, int segmentId)
   {
      segments.add(createSegmentNodeData(seedLabel, segmentId));
   }

   /**
    * iterate computation until there is no more candidate to try merge.
    */
   public SegmentationNodeData createSegmentNodeData(int seedLabel, int segmentId)
   {
      //LogTools.info("createSegmentNodeData " + seedLabel + " " + data.getFusionDataSegment(seedLabel).standardDeviation.getZ());

      FusionDataSegment seedImageSegment = data.getFusionDataSegment(seedLabel);
      SegmentationNodeData newSegment = new SegmentationNodeData(seedImageSegment);

      boolean isPropagating = true;

      while (isPropagating)
      {
         isPropagating = false;

         int[] adjacentLabels = data.getAdjacentLabels(newSegment.labels);
         //LogTools.info("propagating " + adjacentLabels.length);
         for (int adjacentLabel : adjacentLabels)
         {
            //LogTools.info("   candidate label is " + adjacentLabels[i]);
            FusionDataSegment candidate = data.getFusionDataSegment(adjacentLabel);

            if (candidate.isSparse(sparceThreshold))
            {
               //LogTools.info("is too sparce "+candidate.getImageSegmentLabel());
               continue;
            }

            boolean isParallel = false;
            boolean isCoplanar = false;
            if (newSegment.isParallel(candidate, planarityThreshold))
               isParallel = true;
            if (newSegment.isCoplanar(candidate, proximityThreshold))
               isCoplanar = true;

            //LogTools.info("connectivity test result is ## " + (isParallel && isCoplanar) + " ## isParallel " + isParallel + " isCoplanar " + isCoplanar);
            if (isParallel && isCoplanar)
            {
               candidate.setID(segmentId);
               newSegment.merge(candidate);
               isPropagating = true;
            }
         }
      }

      LogTools.info("allLablesInNewSegment");
      TIntArrayList allLablesInNewSegment = newSegment.labels;
      for (int labelNumber : allLablesInNewSegment.toArray())
      {
         LogTools.info("" + labelNumber);
      }

      int[] adjacentLabels = data.getAdjacentLabels(newSegment.labels);
      //LogTools.info("extending for " + adjacentLabels.length + " segments");
      for (int adjacentLabel : adjacentLabels)
      {
         FusionDataSegment adjacentData = data.getFusionDataSegment(adjacentLabel);
         newSegment.extend(adjacentData, extendingPlaneDistanceThreshold, updateNodeDataWithExtendedData);
      }
      return newSegment;
   }

   private int selectRandomNonIdentifiedLabel()
   {
      int randomSeedLabel = -1;
      for (int i = 0; i < maximumNumberOfTrialsToFindUnIdLabel; i++)
      {
         randomSeedLabel = random.nextInt(numberOfLabels - 1);
         if (data.getFusionDataSegment(randomSeedLabel).getId() == -1 && !data.getFusionDataSegment(randomSeedLabel).isSparse(sparceThreshold))
            return randomSeedLabel;
      }
      return -1;
   }

   private class SegmentationNodeData
   {
      TIntArrayList labels = new TIntArrayList();

      final Vector3D normal = new Vector3D();
      final Point3D center = new Point3D();

      double weight = 0.0;

      final List<Point3D> pointsInSegment = new ArrayList<>();

      SegmentationNodeData(FusionDataSegment seedImageSegment)
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

      // TODO : add filtering out far points.
      void extend(FusionDataSegment fusionDataSegment, double threshold, boolean updateNodeData)
      {
         for (Point3D point : fusionDataSegment.getPoints())
         {
            double distance = distancePlaneToPoint(normal, center, point);
            if (distance < threshold)
            {
               int numberOfNeighbors = 0;
               for (Point3D pointInSegment : pointsInSegment)
               {
                  if (pointInSegment.distance(point) < extendingDistanceThreshold)
                  {
                     numberOfNeighbors++;
                  }

                  if (numberOfNeighbors > numberOfNeighborsToExtend)
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
   }

   private static double distancePlaneToPoint(Vector3D normalVector, Point3D center, Point3D point)
   {
      Vector3D centerVector = new Vector3D(center);
      double constantD = -normalVector.dot(centerVector);

      Vector3D pointVector = new Vector3D(point);
      return Math.abs(normalVector.dot(pointVector) + constantD) / Math.sqrt(normalVector.lengthSquared());
   }
}
