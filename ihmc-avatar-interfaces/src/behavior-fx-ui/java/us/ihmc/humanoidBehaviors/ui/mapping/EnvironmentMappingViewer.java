package us.ihmc.humanoidBehaviors.ui.mapping;

import java.io.File;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.hardware.StereoVisionPointCloudDataLoader;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.PlanarRegionFileTools;

public class EnvironmentMappingViewer extends Application
{
   private static final boolean SHOW_PLANAR_REGIONS = false;
   private static final boolean SHOW_STEREO_POINT_CLOUD = true;

   private static final boolean BUILD_PLANAR_REGIONS = false;

   private static final String PLANAR_REGIONS_FILE_NAME = "PlanarRegion";
   private static final String POINT_CLOUD_FILE_NAME = "PointCloud";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      PointCloudGraphic stereoVisionPointCloudGraphic = new PointCloudGraphic();

      File dataFolder = PlanarRegionDataImporter.chooseFile(primaryStage);
      File[] listOfFiles = dataFolder.listFiles();

      if (SHOW_PLANAR_REGIONS)
      {
         File planarRegionsFile = null;
         for (File file : listOfFiles)
         {
            String fileName = file.getName();

            if (fileName.contains(PLANAR_REGIONS_FILE_NAME))
            {
               planarRegionsFile = file;
               break;
            }
         }

         if (planarRegionsFile == null)
         {
            System.out.println("No planar regions file.");
         }
         else
         {
            regionsGraphic.generateMeshes(PlanarRegionFileTools.importPlanarRegionData(planarRegionsFile));
            regionsGraphic.update();
            view3dFactory.addNodeToView(regionsGraphic);
            System.out.println("Planar regions are rendered.");
         }
      }

      if (SHOW_STEREO_POINT_CLOUD)
      {
         File pointCloudFile = null;

         for (File file : listOfFiles)
         {
            String fileName = file.getName();

            if (fileName.contains(POINT_CLOUD_FILE_NAME))
            {
               pointCloudFile = file;
               break;
            }
         }

         if (pointCloudFile == null)
         {
            System.out.println("No point cloud file.");
         }
         else
         {
            List<StereoVisionPointCloudMessage> messagesFromFile = StereoVisionPointCloudDataLoader.getMessagesFromFile(pointCloudFile);
            System.out.println("Point cloud messages (" + messagesFromFile.size() + ")");
            stereoVisionPointCloudGraphic.initializeMeshes();
            stereoVisionPointCloudGraphic.addStereoVisionPointCloudMessageMeshes(messagesFromFile, Color.GREEN);
            stereoVisionPointCloudGraphic.generateMeshes();
            stereoVisionPointCloudGraphic.update();
            view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);
            System.out.println("are rendered.");

            if (BUILD_PLANAR_REGIONS)
            {
               System.out.println("Building planar regions map.");
               List<Long> timestamps = StereoVisionPointCloudDataLoader.extractTimestamps(pointCloudFile);
               EnvironmentMap environmentMap = new EnvironmentMap(timestamps.get(0), messagesFromFile.get(0));
               for (int i = 1; i < messagesFromFile.size(); i++)
               {
                  environmentMap.addFrame(timestamps.get(i), messagesFromFile.get(i));
               }
               regionsGraphic.generateMeshes(environmentMap.getPlanarRegionsMap());
               regionsGraphic.update();
               view3dFactory.addNodeToView(regionsGraphic);
            }
         }
      }

      primaryStage.setTitle(dataFolder.getPath());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
