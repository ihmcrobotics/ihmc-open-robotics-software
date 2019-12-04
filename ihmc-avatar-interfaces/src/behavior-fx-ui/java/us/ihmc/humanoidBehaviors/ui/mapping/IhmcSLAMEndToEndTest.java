package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMEndToEndTest extends Application
{
   private final List<StereoVisionPointCloudMessage> messages = new ArrayList<>();

   //private final String stereoPath = "E:\\Data\\Walking10\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Walking7-fixedframe\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Complicated\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\SimpleArea\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\SimpleArea2\\PointCloud\\";
   private final String stereoPath = "E:\\Data\\SimpleArea3\\PointCloud\\";

   private final boolean showLidarPlanarRegions = false;
   //private final String planarRegionsPath = "E:\\Data\\SimpleArea3\\20191127_222138_PlanarRegion\\";
   private final String planarRegionsPath = "E:\\Data\\Walking7-fixedframe\\PlanarRegions\\";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      messages.addAll(messagesFromFile);
      System.out.println("number of messages " + messages.size());

      IhmcSLAM slam = new IhmcSLAM(true);
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
         slam.addFrame(messages.get(i));

      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      PointCloudGraphic stereoVisionPointCloudGraphic = new PointCloudGraphic();

      stereoVisionPointCloudGraphic.initializeMeshes();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         //stereoVisionPointCloudGraphic.addPointsMeshes(slam.getOriginalPointCloudMap().get(i), slam.getOriginalSensorPoses().get(i), Color.BLACK, Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         //stereoVisionPointCloudGraphic.addPointsMeshes(slam.getPointCloudMap().get(i), slam.getSensorPoses().get(i), Color.BLUE, Color.BLUE);
      }

      stereoVisionPointCloudGraphic.generateMeshes();
      stereoVisionPointCloudGraphic.update();
      view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);

      regionsGraphic.generateMeshes(slam.getPlanarRegionsMap());
      if (showLidarPlanarRegions)
      {
         PlanarRegionsList importPlanarRegionData = PlanarRegionFileTools.importPlanarRegionData(new File(planarRegionsPath));
         RigidBodyTransform preMultiplier = new RigidBodyTransform();
         preMultiplier.appendRollRotation(Math.toRadians(-2.0));
         preMultiplier.setTranslation(-0.1, 0.1, -0.0);
         
         //preMultiplier.appendPitchRotation(Math.toRadians(-5.0));
         importPlanarRegionData.transformByPreMultiply(preMultiplier);
         regionsGraphic.generateMeshes(importPlanarRegionData);
      }
      regionsGraphic.update();
      view3dFactory.addNodeToView(regionsGraphic);

      primaryStage.setTitle(this.getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }

}
