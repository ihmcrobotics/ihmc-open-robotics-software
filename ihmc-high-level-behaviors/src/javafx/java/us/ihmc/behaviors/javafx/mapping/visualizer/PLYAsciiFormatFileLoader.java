package us.ihmc.behaviors.javafx.mapping.visualizer;

import java.io.File;

import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.robotEnvironmentAwareness.slam.tools.PLYasciiFormatFormatDataImporter;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;

public class PLYAsciiFormatFileLoader extends ApplicationNoModule
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      double sizeOfPoint = 0.05;
      
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.05);
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
         Point3D[] pointCloud = PLYasciiFormatFormatDataImporter.getPointsFromFile(pointCloudFile);
         System.out.println("Point cloud [" + pointCloud.length + "] points");
         stereoVisionPointCloudGraphic.initializeMeshes();
         stereoVisionPointCloudGraphic.addPointsMeshes(pointCloud, Color.GREEN, sizeOfPoint);
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
