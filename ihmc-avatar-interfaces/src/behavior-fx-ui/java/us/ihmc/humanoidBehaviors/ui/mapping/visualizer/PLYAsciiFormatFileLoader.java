package us.ihmc.humanoidBehaviors.ui.mapping.visualizer;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;

public class PLYAsciiFormatFileLoader extends Application
{
   public static Point3D[] getPointsFromFile(File dataFile)
   {
      if (!dataFile.canRead())
         new NullPointerException("No dataFile");

      BufferedReader bufferedReader = null;
      try
      {
         System.out.println(dataFile.getAbsolutePath());
         bufferedReader = new BufferedReader(new FileReader(dataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      int numberOfVertex = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] infoArray = null;
         try
         {
            lineJustFetched = bufferedReader.readLine();
            System.out.println(lineJustFetched);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched.contains("format"))
         {
            if (!lineJustFetched.contains("ascii"))
               return null;
         }
         if (lineJustFetched.contains("element vertex"))
         {
            infoArray = lineJustFetched.split(" ");
            numberOfVertex = Integer.parseInt(infoArray[2]);
            System.out.println("number of vertex is " + numberOfVertex);
         }
         if (lineJustFetched.contains("end_header"))
         {
            break;
         }
      }
      Point3D[] pointCloudBuffer = new Point3D[numberOfVertex];
      
      int indexOfPoints = 0;
      while (true)
      {
         String lineJustFetched = null;
         String[] xyzArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched == null)
         {
            break;
         }
         else
         {
            if(indexOfPoints == numberOfVertex)
               break;
            xyzArray = lineJustFetched.split(" ");
            pointCloudBuffer[indexOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]), Double.parseDouble(xyzArray[1]), Double.parseDouble(xyzArray[2]));
            indexOfPoints++;
         }
      }
      Point3D[] points = new Point3D[indexOfPoints];
      for (int i = 0; i < indexOfPoints; i++)
      {
         points[i] = pointCloudBuffer[i];
      }

      return points;
   }

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
         Point3D[] pointCloud = getPointsFromFile(pointCloudFile);
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
