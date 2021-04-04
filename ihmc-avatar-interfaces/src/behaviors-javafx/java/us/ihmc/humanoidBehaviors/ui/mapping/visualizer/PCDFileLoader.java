package us.ihmc.humanoidBehaviors.ui.mapping.visualizer;

import java.io.File;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.slam.tools.PCDDataImporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;

public class PCDFileLoader extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      PointCloudGraphic stereoVisionPointCloudGraphic = new PointCloudGraphic(true);

      File dataFolder = PlanarRegionDataImporter.chooseFile(primaryStage);
      File[] listOfFiles = dataFolder.listFiles();
      File pointCloudFile = listOfFiles[0];

      if (pointCloudFile == null)
      {
         System.out.println("No point cloud file.");
      }
      else
      {
         Point3D[] pointCloud = PCDDataImporter.getPointsFromFile(pointCloudFile);
         if (pointCloud == null)
         {
            System.out.println("no point cloud.");
            return;
         }

         System.out.println("Point cloud [" + pointCloud.length + "] points");
         stereoVisionPointCloudGraphic.initializeMeshes();
         stereoVisionPointCloudGraphic.addPointsMeshes(pointCloud, Color.GREEN);
         stereoVisionPointCloudGraphic.generateMeshes();
         stereoVisionPointCloudGraphic.update();
         view3dFactory.addNodeToView(stereoVisionPointCloudGraphic);
         System.out.println("are rendered.");
      }

      primaryStage.setTitle(pointCloudFile.getPath());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
