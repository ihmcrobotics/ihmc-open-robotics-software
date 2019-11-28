package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;

public class IhmcSLAMEndToEndTest extends Application
{
   private final List<StereoVisionPointCloudMessage> messages = new ArrayList<>();

   //private final String stereoPath = "E:\\Data\\Walking10\\PointCloud\\";
   //private final String stereoPath = "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\";
   private final String stereoPath = "E:\\Data\\Walking7-fixedframe\\PointCloud\\";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(new File(stereoPath));
      messages.addAll(messagesFromFile);
      System.out.println("number of messages " + messages.size());

      IhmcSLAM slam = new IhmcSLAM();
      slam.addFirstFrame(messages.get(0));
      for (int i = 1; i < messages.size(); i++)
      //for (int i = 1; i < 30; i++)
         slam.addFrame(messages.get(i));

      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      StereoVisionPointCloudGraphic stereoVisionPointCloudGraphic = new StereoVisionPointCloudGraphic();

      stereoVisionPointCloudGraphic.initializeMeshes();

      for (int i = 0; i < slam.getOriginalPointCloudMap().size(); i++)
      {
         stereoVisionPointCloudGraphic.addPointsMeshes(slam.getOriginalPointCloudMap().get(i), slam.getOriginalSensorPoses().get(i), Color.BLACK);
      }
      for (int i = 0; i < slam.getPointCloudMap().size(); i++)
      {
         stereoVisionPointCloudGraphic.addPointsMeshes(slam.getPointCloudMap().get(i), slam.getSensorPoses().get(i), Color.BLUE);
      }

      stereoVisionPointCloudGraphic.generateMeshes();
      stereoVisionPointCloudGraphic.update();
      view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);

      regionsGraphic.generateMeshes(slam.getPlanarRegionsMap());
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
