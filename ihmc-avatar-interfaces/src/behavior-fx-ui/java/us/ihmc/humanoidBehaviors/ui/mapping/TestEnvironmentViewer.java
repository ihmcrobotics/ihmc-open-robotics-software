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
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class TestEnvironmentViewer extends Application
{
   private final List<StereoVisionPointCloudMessage> messagesFromFile = new ArrayList<>();
   private final List<Long> timeStampes = new ArrayList<>();
   private final PlanarRegionsList planarRegionList = new PlanarRegionsList();

   private final String stereoPath = "E:\\Data\\Walking9-fixed-warmup\\PointCloud\\";
   private final String extension = ".txt";

   private final List<String> numbersToViz = new ArrayList<>();

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      /**
       * 1574281446221091367 and 1574281446745168115 has discontinuity.
       */
      numbersToViz.add("1574281446221091367");
      numbersToViz.add("1574281446745168115");

      /**
       * 1574281444731577987 and 1574281445344718719 has small overlap area but merge-able.
       */
      //      numbersToViz.add("1574281444731577987");
      //      numbersToViz.add("1574281445344718719");
      //      //numbersToViz.add("1574281445758855178");  // Too close.
      //      //numbersToViz.add("1574281446221091367");  // Too close.
      //      numbersToViz.add("1574281446745168115");

      /**
       * 1574281443383280234 and 1574281443799487886 has big overlap area and merge-able.
       */
      //      numbersToViz.add("1574281443383280234");
      //      numbersToViz.add("1574281443799487886");

      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      PointCloudGraphic stereoVisionPointCloudGraphic = new PointCloudGraphic();

      for (int i = 0; i < numbersToViz.size(); i++)
      {
         String poseName = stereoPath + "pose_" + numbersToViz.get(i) + extension;
         String pointCloudName = stereoPath + "stereo_" + numbersToViz.get(i) + extension;
         messagesFromFile.add(StereoVisionPointCloudDataLoader.getMessageFromFile(new File(poseName), new File(pointCloudName)));
         timeStampes.add(StereoVisionPointCloudDataLoader.extractTimestamp(poseName));
      }

      stereoVisionPointCloudGraphic.initializeMeshes();
      stereoVisionPointCloudGraphic.addStereoVisionPointCloudMessageMesh(messagesFromFile.get(0), Color.RED);
      stereoVisionPointCloudGraphic.addStereoVisionPointCloudMessageMesh(messagesFromFile.get(1), Color.BLUE);
      stereoVisionPointCloudGraphic.generateMeshes();

      stereoVisionPointCloudGraphic.update();
      view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);

      EnvironmentMap environmentMap = new EnvironmentMap(timeStampes.get(0), messagesFromFile.get(0));
      environmentMap.addFrame(timeStampes.get(1), messagesFromFile.get(1));

      regionsGraphic.generateMeshes(environmentMap.getPlanarRegionsMap());
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
