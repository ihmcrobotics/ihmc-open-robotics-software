package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.scene.paint.Color;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMEndToEndTest
{
   private final List<StereoVisionPointCloudMessage> messages = new ArrayList<>();

   //private final String stereoPath = "E:\\Data\\Walking10\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Walking7-fixedframe\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Complicated\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\SimpleArea\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\SimpleArea2\\PointCloud\\";
   private final String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Walking11-kinematic\\PointCloud\\";

   private final boolean showLidarPlanarRegions = false;
   private final String planarRegionsPath = "E:\\Data\\SimpleArea3\\20191127_222138_PlanarRegion\\";
   //private final String planarRegionsPath = "E:\\Data\\Walking7-fixedframe\\PlanarRegions\\";
   //private final String planarRegionsPath = "E:\\Data\\Walking11-kinematic\\20191125_164741_PlanarRegion\\";

   private final boolean doNaiveSLAM = true;

   public IhmcSLAMEndToEndTest()
   {
      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      messages.addAll(messagesFromFile);
      System.out.println("number of messages " + messages.size());

      IhmcSLAM slam = new IhmcSLAM(doNaiveSLAM);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
//      for (int i = 20; i < 60; i++)
         slam.addFrame(messages.get(i));
      if (doNaiveSLAM)
         slam.updatePlanarRegionsSLAM();

      IhmcSLAMViewer slamViewer = new IhmcSLAMViewer();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         //                  slamViewer.addPointCloud(slam.getOriginalPointCloudMap().get(i), Color.BLACK);
         //                  slamViewer.addSensorPose(slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
//         slamViewer.addPointCloud(slam.getPointCloudMap().get(i), Color.BLUE);
//         slamViewer.addSensorPose(slam.getSensorPoses().get(i), Color.BLUE);
      }

      slamViewer.addPlanarRegions(slam.getPlanarRegionsMap());
      if (showLidarPlanarRegions)
      {
         PlanarRegionsList importPlanarRegionData = PlanarRegionFileTools.importPlanarRegionData(new File(planarRegionsPath));
         for (int i = 0; i < importPlanarRegionData.getNumberOfPlanarRegions(); i++)
         {
            importPlanarRegionData.getPlanarRegion(i).setRegionId(0xFF0000);
         }
         slamViewer.addPlanarRegions(importPlanarRegionData);
      }

      slamViewer.start("EndToEnd");
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new IhmcSLAMEndToEndTest();
   }

}
