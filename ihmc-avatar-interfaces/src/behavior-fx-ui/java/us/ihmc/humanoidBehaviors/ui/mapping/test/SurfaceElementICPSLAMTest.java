package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMBasics;
import us.ihmc.robotEnvironmentAwareness.slam.SLAMFrame;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.optimization.FunctionOutputCalculator;
import us.ihmc.robotics.optimization.LevenbergMarquardtParameterOptimizer;

public class SurfaceElementICPSLAMTest
{
   @Test
   public void visualizeFrames()
   {
      /**
       * Frame decision : "..\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\" 4, 5: small drift.
       * <p>
       * 3, 4: small drift on Y dir.
       * <p>
       * 4, 5: yaw drift 6, 7 .........? 2, 3??
       * <p>
       * 7, 8: small drift.
       * <p>
       * 8, 9: medium drift.
       * <p>
       * 9, 10: small drift.
       * <p>
       * 13, 14: big drift and flat overlap (Very difficult).
       * <p>
       * 14, 15: small.
       */
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      //String stereoPath = "C:\\PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\PointCloud\\";   // 8-9 very big in X. 4-5 small in Y. 7-8 big in X and small overlap in the last vertical

      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      for (int i = 0; i < messages.size() - 1; i++)
      {
         SLAMBasics slamBasic = new SLAMBasics(octreeResolution);
         SLAMViewer slamViewer = new SLAMViewer();
         for (int j = 0; j < i; j++)
         {
            slamBasic.addKeyFrame((messages.get(j)));
         }
         slamViewer.addOctree(slamBasic.getOctree(), Color.CORAL, octreeResolution, true);
         slamViewer.addPointCloud(PointCloudCompression.decompressPointCloudToArray(messages.get(i)), Color.CYAN);
         slamViewer.addPointCloud(PointCloudCompression.decompressPointCloudToArray(messages.get(i + 1)), Color.GREEN);
         slamViewer.start(i + " frame and " + (i + 1));
      }

      ThreadTools.sleepForever();
   }

   @Test
   public void testSurfaceElements()
   {
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(3));

      NormalOcTree map = slam.getOctree();
      map.updateNormals();

      SLAMFrame frame2 = new SLAMFrame(slam.getLatestFrame(), messages.get(4));
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.05;
      int minimumNumberOfHits = 10;
      boolean updateNormal = false;
      frame2.registerSurfaceElements(map, windowMargin, surfaceElementResolution, minimumNumberOfHits, updateNormal);

      SLAMViewer slamViewer = new SLAMViewer();
      slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
      slamViewer.addOctree(map, Color.CORAL, octreeResolution);
      slamViewer.addOctree(frame2.getFrameMap(), Color.GREEN, surfaceElementResolution, !updateNormal);

      slamViewer.start("testSurfaceElements");
      ThreadTools.sleepForever();
   }

   @Test
   public void testDistanceComputation()
   {
      //String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      String stereoPath = "C:\\PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(4));

      NormalOcTree map = slam.getOctree();
      map.updateNormals();

      SLAMFrame frame2 = new SLAMFrame(slam.getLatestFrame(), messages.get(5));
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.05;
      int minimumNumberOfHits = 1;
      boolean updateNormal = false;
      frame2.registerSurfaceElements(map, windowMargin, surfaceElementResolution, minimumNumberOfHits, updateNormal);

      int numberOfBigPointsToVisualize = 20;
      TDoubleArrayList bigSurfelDistances = new TDoubleArrayList(numberOfBigPointsToVisualize);
      TIntArrayList bigSurfelIndices = new TIntArrayList(numberOfBigPointsToVisualize);
      for (int i = 0; i < numberOfBigPointsToVisualize; i++)
      {
         bigSurfelDistances.add(0.0);
         bigSurfelIndices.add(0);
      }

      int numberOfSurfel = frame2.getSurfaceElements().size();
      LogTools.info("numberOfSurfel " + numberOfSurfel);
      for (int i = 0; i < numberOfSurfel; i++)
      {
         double distance = SLAMTools.computeDistancePointToNormalOctree(map, frame2.getSurfaceElements().get(i).getPoint());
         System.out.println(distance);
         for (int j = 0; j < bigSurfelDistances.size(); j++)
         {
            double bigDistance = bigSurfelDistances.get(j);
            if (distance > bigDistance)
            {
               for (int k = bigSurfelDistances.size() - 1; k > j; k--)
               {
                  bigSurfelDistances.set(k, bigSurfelDistances.get(k - 1));
                  bigSurfelIndices.set(k, bigSurfelIndices.get(k - 1));
               }
               bigSurfelDistances.set(j, distance);
               bigSurfelIndices.set(j, i);
               break;
            }
         }
      }
      Point3DReadOnly[] bigPoints = new Point3D[numberOfBigPointsToVisualize];
      for (int i = 0; i < bigSurfelDistances.size(); i++)
      {
         bigPoints[i] = new Point3D(frame2.getSurfaceElements().get(bigSurfelIndices.get(i)).getPoint());
         System.out.println(i + " " + bigSurfelIndices.get(i) + " " + bigSurfelDistances.get(i) + " " + bigPoints[i]);
      }

      SLAMViewer slamViewer = new SLAMViewer();
      slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
      slamViewer.addOctree(map, Color.CORAL, octreeResolution);
      slamViewer.addOctree(frame2.getFrameMap(), Color.GREEN, surfaceElementResolution, !updateNormal);
      slamViewer.addPointCloud(bigPoints, Color.RED, 0.02);

      slamViewer.start("testDistanceComputation");
      ThreadTools.sleepForever();
   }

   @Test
   public void testDriftCorrection()
   {
      //String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      String stereoPath = "C:\\PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(14));

      NormalOcTree map = slam.getOctree();
      map.updateNormals();

      SLAMFrame frame2 = new SLAMFrame(slam.getLatestFrame(), messages.get(15));
      double surfaceElementResolution = 0.04;
      double windowMargin = 0.04;
      int minimumNumberOfHits = 3;
      boolean updateNormal = false;
      frame2.registerSurfaceElements(map, windowMargin, surfaceElementResolution, minimumNumberOfHits, updateNormal);

      SLAMViewer originalViewer = new SLAMViewer();
      originalViewer.addOctree(map, Color.CORAL, octreeResolution, true);

      originalViewer.addPointCloud(frame2.getPointCloud(), Color.CYAN);
      originalViewer.addOctree(frame2.getFrameMap(), Color.GREEN, surfaceElementResolution, !updateNormal);
      originalViewer.start("originalViewer");

      int numberOfSurfel = frame2.getSurfaceElementsToSensor().size();
      LogTools.info("numberOfSurfel " + numberOfSurfel);
      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, numberOfSurfel);
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = convertTransform(inputParameter.getData());
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame2.getOriginalSensorPose());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[numberOfSurfel];
            for (int i = 0; i < numberOfSurfel; i++)
            {
               correctedSurfel[i] = new Plane3D();
               correctedSurfel[i].set(frame2.getSurfaceElementsToSensor().get(i));

               correctedSensorPoseToWorld.transform(correctedSurfel[i].getPoint());
               correctedSensorPoseToWorld.transform(correctedSurfel[i].getNormal());
            }

            DenseMatrix64F errorSpace = new DenseMatrix64F(correctedSurfel.length, 1);
            for (int i = 0; i < correctedSurfel.length; i++)
            {
               double distance = computeClosestDistance(correctedSurfel[i]);
               errorSpace.set(i, distance);
            }
            return errorSpace;
         }

         private double computeClosestDistance(Plane3D surfel)
         {
            return SLAMTools.computeBoundedPerpendicularDistancePointToNormalOctree(map, surfel.getPoint(), map.getResolution());
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.0005);
      purterbationVector.set(1, 0.0005);
      purterbationVector.set(2, 0.0005);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      // do ICP.
      for (int i = 0; i < 50; i++)
      {
         optimizer.iterate();
      }

      // get parameter.
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      System.out.println(optimizer.getOptimalParameter());
      icpTransformer.set(convertTransform(optimizer.getOptimalParameter().getData()));
      frame2.updateOptimizedCorrection(icpTransformer);
      System.out.println("icpTransformer");
      System.out.println(icpTransformer);

      Point3DReadOnly[] pointCloud = frame2.getPointCloud();
      RigidBodyTransformReadOnly sensorPose = frame2.getSensorPose();

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = frame2.getPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(pointCloud, sensorPose.getTranslation()));

      map.insertScanCollection(scanCollection, false);
      map.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      map.setNormalEstimationParameters(normalEstimationParameters);
      map.updateNormals();

      SLAMViewer slamViewer = new SLAMViewer();
      slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
      slamViewer.addPointCloud(frame2.getPointCloud(), Color.GREEN);
      slamViewer.addSensorPose(frame2.getSensorPose(), Color.GREEN);
      slamViewer.addSensorPose(frame2.getOriginalSensorPose(), Color.BLUE);
      slamViewer.start("testDriftCorrection");

      ThreadTools.sleepForever();
   }

   private RigidBodyTransform convertTransform(double... transformParameters)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setTranslationAndIdentityRotation(transformParameters[0], transformParameters[1], transformParameters[2]);
      transform.appendRollRotation(transformParameters[3]);
      transform.appendPitchRotation(transformParameters[4]);
      transform.appendYawRotation(transformParameters[5]);

      return transform;
   }

   @Test
   public void testEndToEndTestUpStair2()
   {
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      String planarRegionsPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\20200601_160327_PlanarRegion\\";

      testEndToEnd(stereoPath, planarRegionsPath);
   }

   @Test
   public void testEndToEndTestDownStair()
   {
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_DownStairs\\PointCloud\\";
      String planarRegionsPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_DownStairs\\20200601_154952_PlanarRegion\\";

      testEndToEnd(stereoPath, planarRegionsPath);
   }

   @Test
   public void testEndToEndTestStairUp3()
   {
      String stereoPath = "C:\\PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\PointCloud\\";
      String planarRegionsPath = "C:\\PointCloudData\\Data\\20200603_LidarWalking_StairUp3\\20200603_205049_PlanarRegion\\";

      testEndToEnd(stereoPath, planarRegionsPath);
   }

   private void testEndToEnd(String stereoPath, String planarRegionsPath)
   {
      File pointCloudFile = new File(stereoPath);
      File planarRegionsFile = new File(planarRegionsPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      SLAMViewer originalViewer = new SLAMViewer();
      SLAMViewer slamViewer = new SLAMViewer();
      SLAMViewer octreeViewer = new SLAMViewer();

      originalViewer.addPlanarRegions(PlanarRegionFileTools.importPlanarRegionData(planarRegionsFile));
      octreeViewer.addPlanarRegions(PlanarRegionFileTools.importPlanarRegionData(planarRegionsFile));

      slam.addKeyFrame(messages.get(0));
      slam.updatePlanarRegionsMap();

      for (int i = 1; i < messages.size() - 1; i++)
      {
         slam.addFrame(messages.get(i));
         slam.updatePlanarRegionsMap();
         System.out.println();
         System.out.println(" ## add frame " + i + " " + slam.getComputationTimeForLatestFrame());

         originalViewer.addStereoMessage(messages.get(i), Color.GREEN);
         octreeViewer.addSensorPose(slam.getLatestFrame().getSensorPose());
      }
      slamViewer.addOctree(slam.getOctree(), Color.CORAL, slam.getOctreeResolution(), true);
      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());

      octreeViewer.addOctree(slam.getOctree(), Color.CORAL, slam.getOctreeResolution(), true);

      originalViewer.start("originalViewer");
      slamViewer.start("slamViewer");
      octreeViewer.start("octreeViewer");

      ThreadTools.sleepForever();
   }
}
