package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.SLAMViewer;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.slam.RandomICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAM;
import us.ihmc.robotEnvironmentAwareness.slam.SurfaceElementICPSLAMFrame;
import us.ihmc.robotics.PlanarRegionFileTools;

public class SurfaceElementICPSLAMTest
{
   @Test
   public void visualizeFrames()
   {
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
      /**
       * Frame decision : "..\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\" 4, 5: small drift.
       * <p>
       * 7, 8: small drift.
       * <p>
       * 8, 9: medium drift.
       * <p>
       * 9, 10: small drift.
       * <p>
       * 13, 14: big drift.
       * <p>
       * 14, 15: small and flat overlap.
       */
      String stereoPath = "C:\\PointCloudData\\Data\\20200601_LidarWalking_UpStairs2\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      SurfaceElementICPSLAM slam = new SurfaceElementICPSLAM(octreeResolution);

      SLAMViewer slamViewer = new SLAMViewer();

      slam.addKeyFrame(messages.get(8));
      slam.getOctree().updateNormals();
      SurfaceElementICPSLAMFrame frame2 = new SurfaceElementICPSLAMFrame(slam.getLatestFrame(), messages.get(9));

      double surfaceElementResolution = 0.04;
      frame2.registerSurfaceElements(slam.getOctree(), 0.05, surfaceElementResolution, 15);

      slamViewer.addOctree(slam.getOctree(), Color.CORAL, octreeResolution, true);
      slamViewer.addOctree(slam.getOctree(), Color.CORAL, octreeResolution);

      slamViewer.addOctree(frame2.getSurfaceElements(), Color.GREEN, surfaceElementResolution);

      slamViewer.start("testSurfaceElements");
      ThreadTools.sleepForever();
   }

   @Test
   public void testDriftCorrection()
   {

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
