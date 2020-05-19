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

public class PCDFileLoader extends Application
{
   private enum FieldType
   {
      XYZ, XYZ_NORMAL_CURVATURE;
   }

   public static Point3D[] getPointsFromFile(File dataFile)
   {
      if (!dataFile.canRead())
         new NullPointerException("No dataFile");

      int maximumNumberOfPoints = 200000;
      Point3D[] pointCloudBuffer = new Point3D[maximumNumberOfPoints];
      BufferedReader bufferedReader = null;
      try
      {
         bufferedReader = new BufferedReader(new FileReader(dataFile));
      }
      catch (FileNotFoundException e1)
      {
         e1.printStackTrace();
      }

      boolean isAscii = false;
      FieldType fieldType = FieldType.XYZ;
      while (true)
      {
         String lineJustFetched = null;
         String[] infoArray;
         try
         {
            lineJustFetched = bufferedReader.readLine();
            System.out.println(lineJustFetched);
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
         if (lineJustFetched.contains("FIELDS"))
         {
            infoArray = lineJustFetched.split(" ");
            System.out.println(infoArray.length);
            if (infoArray.length == 4)
            {
               if (infoArray[3].contains("z"))
                  fieldType = FieldType.XYZ;
            }
            else if (infoArray.length == 8)
            {
               if (infoArray[4].contains("normal") && infoArray[7].contains("curvature"))
                  fieldType = FieldType.XYZ_NORMAL_CURVATURE;
            }
            System.out.println("field type is " + fieldType.toString());
         }
         if (lineJustFetched.contains("DATA"))
         {
            if (lineJustFetched.contains("ascii"))
               isAscii = true;
            break;
         }
      }

      if (!isAscii)
      {
         System.out.println("The imported PCD file is not ascii format.");
         return null;
      }
      int numberOfPoints = 0;
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
            if (lineJustFetched.contains("nan"))
               continue;
            xyzArray = lineJustFetched.split(" ");
            switch (fieldType)
            {
               case XYZ:
                  pointCloudBuffer[numberOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]),
                                                                 Double.parseDouble(xyzArray[1]),
                                                                 Double.parseDouble(xyzArray[2]));
                  break;
               case XYZ_NORMAL_CURVATURE:
                  pointCloudBuffer[numberOfPoints] = new Point3D(Double.parseDouble(xyzArray[0]),
                                                                 Double.parseDouble(xyzArray[1]),
                                                                 Double.parseDouble(xyzArray[2]));
                  Double.parseDouble(xyzArray[3]);
                  Double.parseDouble(xyzArray[4]);
                  Double.parseDouble(xyzArray[5]);
                  Double.parseDouble(xyzArray[6]);
                  break;
            }

            numberOfPoints++;
         }
      }
      Point3D[] points = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         points[i] = pointCloudBuffer[i];
      }

      return points;
   }

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
         Point3D[] pointCloud = getPointsFromFile(pointCloudFile);
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
