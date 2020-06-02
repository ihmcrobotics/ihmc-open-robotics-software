package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAMFrame;
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
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);

      for (int i = 0; i < messages.size() - 1; i++)
      {
         SLAMViewer slamViewer = new SLAMViewer();
         slamViewer.addPointCloud(PointCloudCompression.decompressPointCloudToArray(messages.get(i)), Color.BLUE);
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
      slam.addKeyFrame(messages.get(8));

      NormalOcTree map = slam.getOctree();
      map.updateNormals();

      SurfaceElementICPSLAMFrame frame2 = new SurfaceElementICPSLAMFrame(slam.getLatestFrame(), messages.get(9));
      double surfaceElementResolution = 0.04;
      frame2.registerSurfaceElements(map, 0.05, surfaceElementResolution, 15);

      SLAMViewer slamViewer = new SLAMViewer();
      slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
      slamViewer.addOctree(map, Color.CORAL, octreeResolution);

      slamViewer.addOctree(frame2.getSurfaceElements(), Color.GREEN, surfaceElementResolution);

      slamViewer.start("testSurfaceElements");
      ThreadTools.sleepForever();
   }

   @Test
   public void testDriftCorrection()
   {
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);
      slam.addKeyFrame(messages.get(8));

      NormalOcTree map = slam.getOctree();
      map.updateNormals();

      SurfaceElementICPSLAMFrame frame2 = new SurfaceElementICPSLAMFrame(slam.getLatestFrame(), messages.get(9));
      double surfaceElementResolution = 0.04;
      frame2.registerSurfaceElements(map, 0.05, surfaceElementResolution, 10);
      
      SLAMViewer originalViewer = new SLAMViewer();
      originalViewer.addOctree(map, Color.CORAL, octreeResolution, true);

      originalViewer.addPointCloud(frame2.getPointCloud(), Color.BLUE);
      originalViewer.addOctree(frame2.getSurfaceElements(), Color.GREEN, surfaceElementResolution);
      originalViewer.start("originalViewer");

      LevenbergMarquardtParameterOptimizer optimizer = new LevenbergMarquardtParameterOptimizer(6, frame2.getSurfaceElementsToSensor().size());
      FunctionOutputCalculator functionOutputCalculator = new FunctionOutputCalculator()
      {
         @Override
         public DenseMatrix64F computeOutput(DenseMatrix64F inputParameter)
         {
            RigidBodyTransform driftCorrectionTransform = convertTransform(inputParameter.getData());
            RigidBodyTransform correctedSensorPoseToWorld = new RigidBodyTransform(frame2.getOriginalSensorPose());
            correctedSensorPoseToWorld.multiply(driftCorrectionTransform);

            Plane3D[] correctedSurfel = new Plane3D[frame2.getSurfaceElementsToSensor().size()];
            for (int i = 0; i < frame2.getSurfaceElementsToSensor().size(); i++)
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
            return SLAMTools.computeSurfaceElementDistanceToNormalOctree(map, surfel);
         }
      };
      DenseMatrix64F purterbationVector = new DenseMatrix64F(6, 1);
      purterbationVector.set(0, 0.0001);
      purterbationVector.set(1, 0.0001);
      purterbationVector.set(2, 0.0001);
      purterbationVector.set(3, 0.0001);
      purterbationVector.set(4, 0.0001);
      purterbationVector.set(5, 0.0001);
      optimizer.setPerturbationVector(purterbationVector);
      optimizer.setOutputCalculator(functionOutputCalculator);
      optimizer.initialize();
      optimizer.setCorrespondenceThreshold(0.05);

      // do ICP.
      for(int i=0;i<10;i++)
      {
         optimizer.iterate();
      }

      // get parameter.
      RigidBodyTransform icpTransformer = new RigidBodyTransform();
      icpTransformer.set(convertTransform(optimizer.getOptimalParameter().getData()));
      frame2.updateOptimizedCorrection(icpTransformer);
      System.out.println("icpTransformer");
      System.out.println(icpTransformer);
      
      SLAMViewer slamViewer = new SLAMViewer();
      slamViewer.addOctree(map, Color.CORAL, octreeResolution, true);
      slamViewer.addPointCloud(frame2.getPointCloud(), Color.GREEN);
      slamViewer.addOctree(frame2.getSurfaceElements(), Color.GREEN, surfaceElementResolution);
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
   public void testOldSLAMEndToEndTest()
   {
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      RandomICPSLAM slam = new RandomICPSLAM(octreeResolution);
      SLAMViewer originalViewer = new SLAMViewer();
      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
      {
         System.out.println();
         System.out.println(" ## add frame " + i);
         slam.addFrame(messages.get(i));
         slam.updatePlanarRegionsMap();

         originalViewer.addStereoMessage(messages.get(i), Color.GREEN);
      }
      slamViewer.addOctree(slam.getOctree(), Color.CORAL, slam.getOctreeResolution(), true);
      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());

      slamViewer.start("slamViewer");
      originalViewer.start("originalViewer");

      SLAMViewer octreeViewer = new SLAMViewer();

      String path = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\20200601_160327_PlanarRegion\\";
      File file = new File(path);
      octreeViewer.addPlanarRegions(PlanarRegionFileTools.importPlanarRegionData(file));
      octreeViewer.addOctree(slam.getOctree(), Color.CORAL, slam.getOctreeResolution(), true);
      octreeViewer.start("octreeViewer");

      ThreadTools.sleepForever();
   }
}
