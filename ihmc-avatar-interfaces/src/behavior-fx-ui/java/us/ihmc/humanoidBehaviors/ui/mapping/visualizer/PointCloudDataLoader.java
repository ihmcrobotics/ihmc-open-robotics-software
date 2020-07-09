package us.ihmc.humanoidBehaviors.ui.mapping.visualizer;

import java.io.File;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.StereoVisionPointCloudDataLoader;
import us.ihmc.robotics.PlanarRegionFileTools;

public class PointCloudDataLoader extends Application
{
   private static final boolean SHOW_PLANAR_REGIONS = true;
   private static final boolean SHOW_STEREO_POINT_CLOUD = true;
   private static final boolean SHOW_FRAME_BY_FRAME = true;

   private static final String PLANAR_REGIONS_FILE_NAME = "PlanarRegion";
   private static final String POINT_CLOUD_FILE_NAME = "PointCloud";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();
      view3dFactory.setBackgroundColor(Color.WHITE);

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      PointCloudGraphic stereoVisionPointCloudGraphic = new PointCloudGraphic(true);

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
            if (SHOW_FRAME_BY_FRAME)
            {
               Random random = new Random();
               for (int i = 0; i < messagesFromFile.size(); i++)
               {
                  double randomRed = random.nextDouble();
                  double randomGreen = random.nextDouble();
                  double randomBlue = random.nextDouble();
                  stereoVisionPointCloudGraphic.addStereoVisionPointCloudMessageMesh(messagesFromFile.get(i), Color.color(randomRed, randomGreen, randomBlue));
               }
            }
            else
            {
               stereoVisionPointCloudGraphic.addStereoVisionPointCloudMessageMeshes(messagesFromFile, Color.GREEN);
            }
            stereoVisionPointCloudGraphic.generateMeshes();
            stereoVisionPointCloudGraphic.update();
            view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);
            System.out.println("are rendered.");
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
