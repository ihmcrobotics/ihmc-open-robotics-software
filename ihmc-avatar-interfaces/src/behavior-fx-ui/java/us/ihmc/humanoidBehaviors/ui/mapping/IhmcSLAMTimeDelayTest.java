package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TIntArrayList;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.jOctoMap.tools.OcTreeKeyConversionTools;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.updaters.AdaptiveRayMissProbabilityUpdater;

public class IhmcSLAMTimeDelayTest
{
   @Test
   public void testFrameDistanceComputation()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      int referenceFrameIndex = 51;
      int frameIndex = 52;
      StereoVisionPointCloudMessage referenceMessage = messages.get(referenceFrameIndex);
      StereoVisionPointCloudMessage message = messages.get(frameIndex);

      RigidBodyTransform referenceSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(referenceMessage);
      Point3D[] referencePointCloud = IhmcSLAMTools.extractPointsFromMessage(referenceMessage);

      RigidBodyTransform sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);
      Point3D[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);

      // compute NormalOctree.
      double octreeResolution = 0.02;
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = referencePointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(referencePointCloud, referenceSensorPose.getTranslation()));

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
            Point3D selectedPoint = pointCloud[selectedIndex];

            inverseTransformer.transform(selectedPoint, convertedPoint);
            if(convertedPoint.getZ() > 0.7)
            {
               if (-0.1 < convertedPoint.getX() && convertedPoint.getX() < 0.1)
               {
                  if (-0.1 < convertedPoint.getY() && convertedPoint.getY() < 0.1)
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
            System.out.println(distance + " " + searchingSize);
            totalDistance = totalDistance + distance * distance;
            numberOfInliers++;
         }
      }

      System.out.println("avg = " + totalDistance / numberOfInliers + ", numberOfInliers " + numberOfInliers);
   }

   @Test
   public void testOccupancyAssociation()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      StereoVisionPointCloudMessage message = messages.get(51);

      RigidBodyTransform sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);
      Point3D[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);

      // compute NormalOctree.
      double octreeResolution = 0.02;
      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = pointCloud.length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(IhmcSLAMTools.toScan(pointCloud, sensorPose.getTranslation()));

      NormalOcTree octree = new NormalOcTree(octreeResolution);

      octree.insertScanCollection(scanCollection, false);

      octree.enableParallelComputationForNormals(true);
      octree.enableParallelInsertionOfMisses(true);
      octree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      octree.setNormalEstimationParameters(normalEstimationParameters);

      // compute occupancy for a given point.
      Point3D[] points = new Point3D[1];
      points[0] = new Point3D(0.75, 0.0, 0.1);

      int treeDepth = octree.getTreeDepth();
      OcTreeKey occupiedKey = octree.coordinateToKey(points[0]);
      Point3D keyToCoordinate = OcTreeKeyConversionTools.keyToCoordinate(occupiedKey, octreeResolution, treeDepth);
      System.out.println(occupiedKey.getKey(0) + " " + occupiedKey.getKey(1) + " " + occupiedKey.getKey(2));
      System.out.println(keyToCoordinate);

      int searchingSize = 3;
      int lengthOfBox = 1 + searchingSize * 2;
      int numberOfKeyInBox = lengthOfBox * lengthOfBox * lengthOfBox;
      Point3D[] boxKeyPoints = new Point3D[numberOfKeyInBox];
      Point3D[] closestPoint = new Point3D[1];
      closestPoint[0] = new Point3D();
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

               NormalOcTreeNode searchNode = octree.search(dummyKey);
               if (searchNode != null)
               {
                  Point3D dummyPoint = OcTreeKeyConversionTools.keyToCoordinate(dummyKey, octreeResolution, treeDepth);
                  boxKeyPoints[i * lengthOfBox * lengthOfBox + j * lengthOfBox + k] = new Point3D(dummyPoint);

                  double distance = dummyPoint.distance(points[0]);
                  if (distance < minDistance)
                  {
                     minDistance = distance;
                     closestPoint[0].set(dummyPoint);
                  }
               }
               else
               {
                  boxKeyPoints[i * lengthOfBox * lengthOfBox + j * lengthOfBox + k] = new Point3D();
                  continue;
               }
            }
         }
      }

      // visualize octree and the given point.
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      viewer.addPointCloud(pointCloud, Color.BLUE);
      viewer.addPointCloud(points, Color.RED);
      viewer.addPointCloud(boxKeyPoints, Color.ALICEBLUE);
      viewer.addPointCloud(closestPoint, Color.CORAL);

      viewer.start("testOccupancyAssociation");

      ThreadTools.sleepForever();
   }

   @Test
   public void testIhmcSLAM()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      IhmcSLAM slam = new IhmcSLAM(true);
      slam.addFirstFrame(messages.get(47));
      for (int i = 40; i < 70; i++)
      {
         slam.addFrame(messages.get(i));
      }

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         //         viewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.BLACK);
         //         viewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         viewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         viewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }

      //      viewer.addPlanarRegions(slam.getPlanarRegionsMap());
      viewer.start("EndToEnd");

      ThreadTools.sleepForever();
   }

   @Test
   public void testSimilarityEndToEnd()
   {
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      int numberOfTrials = 30;
      double octreeResolution = 0.02;
      for (int i = 1; i < 80; i++)
      {
         StereoVisionPointCloudMessage previousMessage = messagesFromFile.get(i - 1);
         StereoVisionPointCloudMessage message = messagesFromFile.get(i);

         RigidBodyTransform previousSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(previousMessage);
         Point3D[] previousPointCloud = IhmcSLAMTools.extractPointsFromMessage(previousMessage);

         RigidBodyTransform sensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(message);
         Point3D[] pointCloud = IhmcSLAMTools.extractPointsFromMessage(message);

         double bestAlpha = IhmcSLAMTools.computeOptimizedAlpha(sensorPose, pointCloud, previousSensorPose, previousPointCloud, numberOfTrials,
                                                                octreeResolution);

         RigidBodyTransform newSensorPose = IhmcSLAMTools.createTimeDelayedSensorPose(sensorPose, previousSensorPose, bestAlpha);
         Point3D[] newPointCloud = IhmcSLAMTools.createConvertedPointsToWorld(sensorPose, pointCloud, newSensorPose);

         viewer.addSensorPose(newSensorPose, Color.BLUE);
         viewer.addPointCloud(newPointCloud, Color.BLUE);
         System.out.println(i + " is done. " + bestAlpha);
      }

      viewer.start("testSimilarity");
      ThreadTools.sleepForever();
   }

   @Test
   public void testTimeDelay()
   {
      double assumedAlpha = 0.3;
      boolean doTimeDelay = false;

      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      // 53-54 was great.
      // 50-51: 0.15 
      int previousIndex = 51;
      int nextIndex = 52;

      StereoVisionPointCloudMessage previousMessage = messagesFromFile.get(previousIndex);
      StereoVisionPointCloudMessage nextMessage = messagesFromFile.get(nextIndex);

      // A: naive.
      if (!doTimeDelay)
      {
         viewer.addStereoMessage(previousMessage, Color.GREEN, Color.GREEN);
         viewer.addStereoMessage(nextMessage, Color.BLUE, Color.BLUE);
      }

      // B: time compensated.
      else
      {
         RigidBodyTransform previousSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(previousMessage);
         Point3D[] previousPointCloud = IhmcSLAMTools.extractPointsFromMessage(previousMessage);

         RigidBodyTransform nextSensorPose = IhmcSLAMTools.extractSensorPoseFromMessage(nextMessage);
         Point3D[] nextPointCloud = IhmcSLAMTools.extractPointsFromMessage(nextMessage);

         RigidBodyTransform newSensorPose = IhmcSLAMTools.createTimeDelayedSensorPose(nextSensorPose, previousSensorPose, assumedAlpha);
         Point3D[] newPointCloud = IhmcSLAMTools.createConvertedPointsToWorld(nextSensorPose, nextPointCloud, newSensorPose);

         viewer.addSensorPose(previousSensorPose, Color.GREEN);
         viewer.addSensorPose(nextSensorPose, Color.BLUE);
         viewer.addSensorPose(newSensorPose, Color.RED);

         viewer.addPointCloud(previousPointCloud, Color.GREEN);
         viewer.addPointCloud(nextPointCloud, Color.BLUE);
         viewer.addPointCloud(newPointCloud, Color.RED);
      }

      viewer.start("testTimeDelay " + doTimeDelay + " " + assumedAlpha + " " + previousIndex + " " + nextIndex);
      ThreadTools.sleepForever();
   }

   @Test
   public void testTimeDelayStacked()
   {
      IhmcSLAMViewer viewer = new IhmcSLAMViewer();
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      // 49 is bad
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(47), messages.get(46), 0.6), Color.GREEN);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(48), messages.get(47), 0.6), Color.RED);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(49), messages.get(48), 0.1), Color.BLUE);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(50), messages.get(49), 0.9), Color.BLUE);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(51), messages.get(50), 0.1), Color.BLUE);

      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(52), messages.get(51), 0.7), Color.BLUE);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(53), messages.get(52), 0.9), Color.BLUE);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(54), messages.get(53), 0.9), Color.BLUE);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(55), messages.get(54), 0.9), Color.BLUE);
      viewer.addPointCloud(IhmcSLAMTools.createTimeDelayedPoints(messages.get(56), messages.get(55), 0.5), Color.BLUE);
      viewer.start("testTimeDelayStacked ");
      ThreadTools.sleepForever();
   }
}
