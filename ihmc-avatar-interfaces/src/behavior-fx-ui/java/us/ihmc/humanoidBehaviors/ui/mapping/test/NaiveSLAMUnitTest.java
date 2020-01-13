package us.ihmc.humanoidBehaviors.ui.mapping.test;

import java.io.File;
import java.util.List;

import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.IhmcSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.NaiveSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.visualizer.IhmcSLAMViewer;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;

public class NaiveSLAMUnitTest
{
   @Test
   public void testNaiveSLAMEndToEnd()
   {
      String stereoPath = "E:\\Data\\20200108_Normal Walk\\PointCloud\\";
      File pointCloudFile = new File(stereoPath);

      List<StereoVisionPointCloudMessage> messages = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
      double octreeResolution = 0.02;
      IhmcSLAM slam = new NaiveSLAM(octreeResolution);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
      {
         System.out.println(" ## add frame " + i);
         slam.addFrame(messages.get(i));
      }

      slam.updatePlanarRegionsMap();

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
         slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }
      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());

      slamViewer.start("testNaiveSLAMEndToEnd");
      ThreadTools.sleepForever();
   }
}
