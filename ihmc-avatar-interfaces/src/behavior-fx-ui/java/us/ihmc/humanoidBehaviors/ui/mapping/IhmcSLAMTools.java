package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAMFrame;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;

public class IhmcSLAMTools
{
   public static Point3D[] extractPointsFromMessage(StereoVisionPointCloudMessage message)
   {
      int numberOfPoints = message.getColors().size();
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = new Point3D();
         MessageTools.unpackScanPoint(message, i, pointCloud[i]);
      }
      return pointCloud;
   }

   public static RigidBodyTransform extractSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }

   public static Point3D[] createConvertedPointsToSensorPose(RigidBodyTransformReadOnly sensorPose, Point3DReadOnly[] pointCloud)
   {
      Point3D[] convertedPoints = new Point3D[pointCloud.length];
      RigidBodyTransform inverseTransformer = new RigidBodyTransform(sensorPose);
      inverseTransformer.invert();
      Point3D convertedPoint = new Point3D();
      for (int i = 0; i < pointCloud.length; i++)
      {
         inverseTransformer.transform(pointCloud[i], convertedPoint);
         convertedPoints[i] = new Point3D(convertedPoint);
      }

      return convertedPoints;
   }

   public static Point3D[] createConvertedPointsToOtherSensorPose(RigidBodyTransformReadOnly sensorPose, RigidBodyTransformReadOnly otherSensorPose,
                                                                  Point3DReadOnly[] pointCloudToSensorPose)
   {
      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensorPose.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensorPose[i]);
         sensorPose.transform(pointCloudToWorld[i]);
      }

      return createConvertedPointsToSensorPose(otherSensorPose, pointCloudToWorld);
   }

   public static Point3D[] createConvertedPointsToWorld(RigidBodyTransformReadOnly otherSensorPose, Point3DReadOnly[] pointCloudToSensorPose)
   {
      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensorPose.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensorPose[i]);
         otherSensorPose.transform(pointCloudToWorld[i]);
      }

      return pointCloudToWorld;
   }

   public static Point3D[] createConvertedPointsToWorld(RigidBodyTransformReadOnly sensorPose, Point3DReadOnly[] pointCloudToWorld,
                                                        RigidBodyTransformReadOnly otherSensorPose)
   {
      Point3D[] pointCloudToSensorPose = createConvertedPointsToSensorPose(sensorPose, pointCloudToWorld);

      return createConvertedPointsToWorld(otherSensorPose, pointCloudToSensorPose);
   }

   public static RigidBodyTransform createTimeDelayedSensorPose(RigidBodyTransformReadOnly sensorPose, RigidBodyTransformReadOnly previousSensorPose,
                                                                double alphaFromPrevious)
   {
      Tuple3DReadOnly previousSensorPosition = previousSensorPose.getTranslation();
      Quaternion previousOrientation = new Quaternion(previousSensorPose.getRotation());

      Point3D newPosition = new Point3D();
      Quaternion newOrientation = new Quaternion();
      newPosition.interpolate(previousSensorPosition, sensorPose.getTranslation(), alphaFromPrevious);
      newOrientation.interpolate(previousOrientation, new Quaternion(sensorPose.getRotation()), alphaFromPrevious);

      RigidBodyTransform newSensorPose = new RigidBodyTransform(newOrientation, newPosition);
      return newSensorPose;
   }

   public static Point3D[] createTimeDelayedPoints(Point3DReadOnly[] pointCloudToWorld, RigidBodyTransformReadOnly sensorPose,
                                                   RigidBodyTransformReadOnly previousSensorPose, double alphaFromPrevious)
   {
      RigidBodyTransform newSensorPose = createTimeDelayedSensorPose(sensorPose, previousSensorPose, alphaFromPrevious);
      Point3D[] newPointCloud = IhmcSLAMTools.createConvertedPointsToWorld(sensorPose, pointCloudToWorld, newSensorPose);
      return newPointCloud;
   }

   public static RigidBodyTransform createTimeDelayedSensorPose(StereoVisionPointCloudMessage message, StereoVisionPointCloudMessage previousMessage,
                                                                double alphaFromPrevious)
   {
      RigidBodyTransform previousSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(previousMessage);
      RigidBodyTransform sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      return createTimeDelayedSensorPose(sensorPose, previousSensorPose, alphaFromPrevious);
   }

   public static Point3D[] createTimeDelayedPoints(StereoVisionPointCloudMessage message, StereoVisionPointCloudMessage previousMessage,
                                                   double alphaFromPrevious)
   {
      Point3D[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);
      RigidBodyTransform sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);
      RigidBodyTransform previousSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(previousMessage);

      return createTimeDelayedPoints(pointCloud, sensorPose, previousSensorPose, alphaFromPrevious);
   }

   public static Scan toScan(Point3DReadOnly[] points, Tuple3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      for (int i = 0; i < points.length; i++)
      {
         double x = points[i].getX();
         double y = points[i].getY();
         double z = points[i].getZ();
         pointCloud.add(x, y, z);
      }
      return new Scan(new Point3D(sensorPosition), pointCloud);
   }

   public static NormalOcTree computeOctreeData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(toScan(pointCloud, sensorPosition));

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();
      return referenceOctree;
   }

   public static NormalOcTree computeOctreeData(List<Point3DReadOnly[]> pointCloudMap, List<RigidBodyTransformReadOnly> sensorPoses, double octreeResolution)
   {
      ScanCollection scanCollection = new ScanCollection();
      for (int i = 0; i < pointCloudMap.size(); i++)
      {
         int numberOfPoints = pointCloudMap.get(i).length;

         scanCollection.setSubSampleSize(numberOfPoints);
         scanCollection.addScan(toScan(pointCloudMap.get(i), sensorPoses.get(i).getTranslation()));
      }

      NormalOcTree referenceOctree = new NormalOcTree(octreeResolution);

      referenceOctree.insertScanCollection(scanCollection, false);

      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      referenceOctree.setNormalEstimationParameters(normalEstimationParameters);

      referenceOctree.updateNormals();
      return referenceOctree;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(List<Point3DReadOnly[]> pointCloudMap,
                                                                                  List<RigidBodyTransformReadOnly> sensorPoses, double octreeResolution,
                                                                                  PlanarRegionSegmentationParameters planarRegionSegmentationParameters)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloudMap, sensorPoses, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(false);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(new Point3D(0.0, 0.0, 20.0)); //TODO: work this for every poses.

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition,
                                                                                  double octreeResolution,
                                                                                  PlanarRegionSegmentationParameters planarRegionSegmentationParameters)
   {
      return computePlanarRegionRawData(pointCloud, sensorPosition, octreeResolution, planarRegionSegmentationParameters, true);
   }

   public static List<PlanarRegionSegmentationRawData> computePlanarRegionRawData(Point3DReadOnly[] pointCloud, Tuple3DReadOnly sensorPosition,
                                                                                  double octreeResolution,
                                                                                  PlanarRegionSegmentationParameters planarRegionSegmentationParameters,
                                                                                  boolean useSurfaceNormalFilter)
   {
      NormalOcTree referenceOctree = computeOctreeData(pointCloud, sensorPosition, octreeResolution);

      PlanarRegionSegmentationCalculator segmentationCalculator = new PlanarRegionSegmentationCalculator();

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(useSurfaceNormalFilter);

      segmentationCalculator.setParameters(planarRegionSegmentationParameters);
      segmentationCalculator.setSurfaceNormalFilterParameters(surfaceNormalFilterParameters);
      segmentationCalculator.setSensorPosition(sensorPosition);

      segmentationCalculator.compute(referenceOctree.getRoot());

      List<PlanarRegionSegmentationRawData> rawData = segmentationCalculator.getSegmentationRawData();

      return rawData;
   }

   public static double computeSimilarity(Point3DReadOnly[] previousPointCloud, RigidBodyTransform previousSensorPose, Point3DReadOnly[] pointCloud,
                                          RigidBodyTransform sensorPose, double octreeResolution)
   {
      // compute NormalOctree.
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = previousPointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(previousPointCloud, previousSensorPose.getTranslation()));

      NormalOcTree octree = new NormalOcTree(octreeResolution);

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // Select source points.
      RigidBodyTransform inverseTransformer = new RigidBodyTransform(sensorPose);
      inverseTransformer.invert();
      Point3D convertedPoint = new Point3D();

      double depthThreshold = 0.7;
      double windowWidth = 0.3;
      double windowHeight = 0.2;
      int numberOfSourcePoints = 100;
      int numberOfInliers = 0;
      TIntArrayList indexOfSourcePoints = new TIntArrayList();
      Point3D[] sourcePoints = new Point3D[numberOfSourcePoints];
      Random randomSelector = new Random(0612L);
      while (indexOfSourcePoints.size() != numberOfSourcePoints)
      {
         int selectedIndex = randomSelector.nextInt(pointCloud.length);
         if (!indexOfSourcePoints.contains(selectedIndex))
         {
            Point3DReadOnly selectedPoint = pointCloud[selectedIndex];

            inverseTransformer.transform(selectedPoint, convertedPoint);
            if (convertedPoint.getZ() > depthThreshold)
            {
               if (-windowWidth / 2 < convertedPoint.getX() && convertedPoint.getX() < windowWidth / 2)
               {
                  if (-windowHeight / 2 < convertedPoint.getY() && convertedPoint.getY() < windowHeight / 2)
                  {
                     indexOfSourcePoints.add(selectedIndex);
                  }
               }
            }
         }
      }
      for (int i = 0; i < indexOfSourcePoints.size(); i++)
      {
         sourcePoints[i] = new Point3D(pointCloud[indexOfSourcePoints.get(i)]);
      }

      // Compute distance.
      double totalDistance = 0;
      for (Point3D sourcePoint : sourcePoints)
      {
         int searchingSize = 3;
         double distance = -1.0;
         while (distance < 0 && searchingSize < 6)
         {
            distance = IhmcSLAMTools.computeDistanceToNormalOctree(octree, sourcePoint, searchingSize);
            searchingSize++;
         }

         if (distance > 0)
         {
            totalDistance = totalDistance + distance * distance;
            numberOfInliers++;
         }
      }

      return totalDistance / numberOfInliers;
   }

   public static double computeOptimizedAlpha(RigidBodyTransform sensorPose, Point3D[] pointCloud, RigidBodyTransform previousSensorPose,
                                              Point3D[] previousPointCloud, int numberOfTrials, double octreeResolution)
   {
      double bestAlpha = -1.0;
      double minScore = Double.MAX_VALUE;
      for (int i = 0; i < numberOfTrials; i++)
      {
         double assumedAlpha = (double) (i + 1) / numberOfTrials;
         RigidBodyTransform newSensorPose = IhmcSLAMTools.createTimeDelayedSensorPose(sensorPose, previousSensorPose, assumedAlpha);
         Point3D[] newPointCloud = IhmcSLAMTools.createConvertedPointsToWorld(sensorPose, pointCloud, newSensorPose);

         double score = IhmcSLAMTools.computeSimilarity(previousPointCloud, previousSensorPose, newPointCloud, newSensorPose, octreeResolution);

         if (score < minScore)
         {
            bestAlpha = assumedAlpha;
            minScore = score;
         }
      }
      return bestAlpha;
   }

   public static List<Point3D> closestOctreePoints = new ArrayList<>();
   public static List<Point3D> duplicatedPoints = new ArrayList<>();
   public static List<Point3D> candidatePoints = new ArrayList<>();

   public static double computeDistanceToNormalOctree(NormalOcTree octree, Point3DReadOnly point, int searchingSize)
   {
      double octreeResolution = octree.getResolution();
      int treeDepth = octree.getTreeDepth();

      OcTreeKey occupiedKey = octree.coordinateToKey(point);

      int lengthOfBox = 1 + searchingSize * 2;
      Point3D closestPoint = new Point3D();
      //double minDistance = -1.0;
      double minDistance = Double.MAX_VALUE;
      for (int i = 0; i < lengthOfBox; i++)
      {
         for (int j = 0; j < lengthOfBox; j++)
         {
            for (int k = 0; k < lengthOfBox; k++)
            {
               OcTreeKey dummyKey = new OcTreeKey();
               dummyKey.setKey(0, occupiedKey.getKey(0) - searchingSize + i);
               dummyKey.setKey(1, occupiedKey.getKey(1) - searchingSize + j);
               dummyKey.setKey(2, occupiedKey.getKey(2) - searchingSize + k);

               if (!octree.isInBoundingBox(dummyKey))
               {
                  continue;
               }

               NormalOcTreeNode searchNode = octree.search(dummyKey);
               if (searchNode != null)
               {
                  //                  if (minDistance < 0.0)
                  //                     minDistance = Double.MAX_VALUE;

                  Point3D pointInBox = OcTreeKeyConversionTools.keyToCoordinate(dummyKey, octreeResolution, treeDepth);

                  double distance = pointInBox.distance(point);
                  //double distance = computeDistanceToNormalOctreeNode(searchNode, point);
                  if (distance < minDistance)
                  {
                     minDistance = distance;
                     closestPoint.set(pointInBox);
                  }
               }
               else
               {
                  continue;
               }
            }
         }
      }
      closestOctreePoints.add(closestPoint);
      System.out.println();
      System.out.println("point " + point);
      System.out.println("closestPoint " + closestPoint);

      return minDistance;
   }

   public static double computeDistanceToNormalOctree2(NormalOcTree octree, Point3DReadOnly point, int maximumSearchingSize)
   {
      double octreeResolution = octree.getResolution();
      int treeDepth = octree.getTreeDepth();

      OcTreeKey occupiedKey = octree.coordinateToKey(point);

      Point3D closestPoint = new Point3D();
      double minDistance = -1.0;

      if (octree.search(occupiedKey) != null)
      {
         Point3D candidatePoint = OcTreeKeyConversionTools.keyToCoordinate(occupiedKey, octreeResolution, treeDepth);
         duplicatedPoints.add(candidatePoint);
         return 0.0;
      }

      for (int searchingSize = 1; searchingSize < maximumSearchingSize + 1; searchingSize++)
      {
         OcTreeKey[] candidateKeys = createCandidateOctreeKey(occupiedKey, searchingSize);
         for (int i = 0; i < candidateKeys.length; i++)
         {
            OcTreeKey candidateKey = candidateKeys[i];

            NormalOcTreeNode searchNode = octree.search(candidateKey);
            if (searchNode != null)
            {
               if (minDistance < 0.0)
                  minDistance = Double.MAX_VALUE;

               //Point3D candidatePoint = OcTreeKeyConversionTools.keyToCoordinate(candidateKey, octreeResolution, treeDepth);
               Point3D candidatePoint = new Point3D(searchNode.getX(), searchNode.getY(), searchNode.getZ());

               candidatePoints.add(candidatePoint);

               double distance = candidatePoint.distance(point);
               if (distance < minDistance)
               {
                  minDistance = distance;
                  closestPoint.set(candidatePoint);
               }
            }
            else
            {
               continue;
            }
         }
         if (minDistance > 0)
         {
            break;
         }
      }

      closestOctreePoints.add(closestPoint);

      return minDistance;
   }

   private static OcTreeKey[] createCandidateOctreeKey(OcTreeKey key, int searchingSize)
   {
      int outerSize = 2 * searchingSize + 1;
      int innerSize = 2 * searchingSize - 1;
      int numberOfCandidates = outerSize * outerSize * outerSize - innerSize * innerSize * innerSize;
      OcTreeKey[] candidates = new OcTreeKey[numberOfCandidates];

      int index = 0;
      for (int i = 0; i < outerSize; i++)
      {
         for (int j = 0; j < outerSize; j++)
         {
            for (int k = 0; k < outerSize; k++)
            {
               if (i == 0 || i == outerSize - 1)
               {
                  OcTreeKey candidate = new OcTreeKey();
                  candidate.setKey(0, key.getKey(0) - searchingSize + i);
                  candidate.setKey(1, key.getKey(1) - searchingSize + j);
                  candidate.setKey(2, key.getKey(2) - searchingSize + k);
                  candidates[index] = candidate;
                  index++;
               }
               else
               {
                  if (j == 0 || j == outerSize - 1 || k == 0 || k == outerSize - 1)
                  {
                     OcTreeKey candidate = new OcTreeKey();
                     candidate.setKey(0, key.getKey(0) - searchingSize + i);
                     candidate.setKey(1, key.getKey(1) - searchingSize + j);
                     candidate.setKey(2, key.getKey(2) - searchingSize + k);
                     candidates[index] = candidate;
                     index++;
                  }
               }
            }
         }
      }

      return candidates;
   }

   private static double computeDistanceToNormalOctreeNode(NormalOcTreeNode searchNode, Point3DReadOnly point)
   {
      Plane3D plane = new Plane3D(searchNode.getHitLocationCopy(), searchNode.getNormalCopy());

      return plane.distance(point);
   }

   /**
    * if there is not enough source points, think this frame is key frame.
    * return null.
    */
   public static Point3D[] createSourcePointsToSensorPose(IhmcSLAMFrame frame, int numberOfSourcePoints, ConvexPolygon2D previousWindow, double windowDepth,
                                                          double minimumOverlappedRatio)
   {
      IhmcSLAMFrame previousFrame = frame.getPreviousFrame();

      //List<Point3DReadOnly> pointsOutOfPreviousWindow = new ArrayList<>();
      List<Point3DReadOnly> pointsInPreviousWindow = new ArrayList<>();

      RigidBodyTransformReadOnly previousSensorPoseToWorld = previousFrame.getSensorPose();
      Point3DReadOnly[] originalPointCloudToSensorPose = frame.getOriginalPointCloudToSensorPose();
      Point3DReadOnly[] convertedPointsToPreviousSensorPose = IhmcSLAMTools.createConvertedPointsToOtherSensorPose(frame.getInitialSensorPoseToWorld(),
                                                                                                                   previousSensorPoseToWorld,
                                                                                                                   originalPointCloudToSensorPose);

      int numberOfPointsInPreviousView = 0;
      for (int i = 0; i < convertedPointsToPreviousSensorPose.length; i++)
      {
         Point3DReadOnly pointInPreviousView = convertedPointsToPreviousSensorPose[i];
         if (previousWindow.isPointInside(pointInPreviousView.getX(), pointInPreviousView.getY()) && pointInPreviousView.getZ() > windowDepth)
         {
            pointsInPreviousWindow.add(originalPointCloudToSensorPose[i]);
            numberOfPointsInPreviousView++;
         }
         else
         {
            //pointsOutOfPreviousWindow.add(originalPointCloudToSensorPose[i]);
         }
      }
      double overlappedRatio = (double) numberOfPointsInPreviousView / convertedPointsToPreviousSensorPose.length;
      if (overlappedRatio < minimumOverlappedRatio)
      {
         return null;
      }
      if (pointsInPreviousWindow.size() < numberOfSourcePoints)
      {
         return null;
      }

      TIntArrayList indexOfSourcePoints = new TIntArrayList();
      int index = 0;
      Point3D[] sourcePoints = new Point3D[numberOfSourcePoints];
      Random randomSelector = new Random(0612L);
      while (indexOfSourcePoints.size() != numberOfSourcePoints)
      {
         int selectedIndex = randomSelector.nextInt(pointsInPreviousWindow.size());
         if (!indexOfSourcePoints.contains(selectedIndex))
         {
            Point3DReadOnly selectedPoint = pointsInPreviousWindow.get(selectedIndex);
            sourcePoints[index] = new Point3D(selectedPoint);
            index++;
            indexOfSourcePoints.add(selectedIndex);
         }
      }

      System.out.println(convertedPointsToPreviousSensorPose.length + " " + numberOfPointsInPreviousView + " " + overlappedRatio);

      return sourcePoints;
   }

   public static int countNumberOfInliers(NormalOcTree octree, RigidBodyTransformReadOnly sensorPoseToWorld, Point3DReadOnly[] sourcePointsToSensor,
                                          int maximumSearchingSize)
   {
      int numberOfInliers = 0;
      Point3D newSourcePointToWorld = new Point3D();
      for (Point3DReadOnly sourcePoint : sourcePointsToSensor)
      {
         newSourcePointToWorld.set(sourcePoint);
         sensorPoseToWorld.transform(newSourcePointToWorld);

         double distance = IhmcSLAMTools.computeDistanceToNormalOctree2(octree, newSourcePointToWorld, maximumSearchingSize);

         if (distance >= 0)
         {
            if (distance < octree.getResolution())
            {
               numberOfInliers++;
            }
         }
      }
      return numberOfInliers;
   }
}
