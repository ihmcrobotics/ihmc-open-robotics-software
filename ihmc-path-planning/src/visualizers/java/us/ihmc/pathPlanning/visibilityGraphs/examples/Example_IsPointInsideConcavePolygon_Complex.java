package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityTools;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_IsPointInsideConcavePolygon_Complex extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();

   double extrusionDistance = 0.60;

   public Example_IsPointInsideConcavePolygon_Complex()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      File file = new File("C:\\Users\\WOPR-M\\repository\\ihmc-path-planning\\src\\visualizers\\resources\\Data\\20171121_103600_PlanarRegion");

      PlanarRegionsList planarRegionData = null;
      planarRegionData = PlanarRegionDataImporter.importPlanRegionData(file);

      //      PlanarRegion region = planarRegionData.getPlanarRegion(3);

      for (PlanarRegion region : planarRegionData.getPlanarRegionsAsList())
      {
         Point2D pointToCheck = new Point2D(-2, -0.4);

         ArrayList<Point2D> points = new ArrayList<>();
         for (int i = 1; i < region.getConcaveHullSize(); i++)
         {
            Point2D point = region.getConcaveHull()[i];
            points.add(point);
         }
         points.add(points.get(0));

         for (int i = 1; i < points.size(); i++)
         {
            javaFXMultiColorMeshBuilder.addLine(new Point3D(points.get(i - 1).getX(), points.get(i - 1).getY(), 0),
                                                new Point3D(points.get(i).getX(), points.get(i).getY(), 0), 0.005, Color.BLACK);
         }

         Point2D centroid = EuclidGeometryTools.averagePoint2Ds(points);

         Vector2D directionToCentroid = new Vector2D(centroid.getX() - pointToCheck.getX(), centroid.getY() - pointToCheck.getY());
         directionToCentroid.normalize();
         directionToCentroid.scale(10);

         Point2D endPoint = new Point2D(pointToCheck.getX() + directionToCentroid.getX(), pointToCheck.getY() + directionToCentroid.getY());

         if (PlanarRegionTools.isPointInsidePolygon(region.getConcaveHull(), pointToCheck))
         {
            System.out.println("Point is inside");
         }
         else
         {
            System.out.println("Point is outside");
         }

         javaFXMultiColorMeshBuilder.addLine(new Point3D(pointToCheck.getX(), pointToCheck.getY(), 0), new Point3D(endPoint.getX(), endPoint.getY(), 0), 0.005,
                                             Color.RED);
      }

      //      for (int i = 1; i < ground.getConcaveHullSize(); i++)
      //      {
      //         javaFXMultiColorMeshBuilder.addLine(new Point3D(ground.getConcaveHull()[i-1].getX(), ground.getConcaveHull()[i-1].getY(), 0),
      //                                             new Point3D(ground.getConcaveHull()[i].getX(), ground.getConcaveHull()[i].getY(), 0), 0.005, Color.BLACK);
      //      }
      //      
      //      Point2D pointToCheck = new Point2D(-2,-0.4);
      //      
      //      ArrayList<Point2D> points = new ArrayList<>();
      //      for (int i = 1; i < ground.getConcaveHullSize(); i++)
      //      {
      //         Point2D point = ground.getConcaveHull()[i];
      //         points.add(point);
      //      }
      //      
      //      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(points);
      //
      //      Vector2D directionToCentroid = new Vector2D(centroid.getX() - pointToCheck.getX(), centroid.getY() - pointToCheck.getY());
      //      directionToCentroid.normalize();
      //      directionToCentroid.scale(10);
      //      
      //      Point2D endPoint = new Point2D(pointToCheck.getX() + directionToCentroid.getX(), pointToCheck.getY() + directionToCentroid.getY());
      //
      //      if(VisibilityTools.isPointInsideConcavePolygon(ground.getConcaveHull(), pointToCheck, endPoint))
      //      {
      //         System.out.println("Point is inside");
      //      }
      //      else
      //      {
      //         System.out.println("Point is outside");
      //      }
      //
      //      javaFXMultiColorMeshBuilder.addLine(new Point3D(pointToCheck.getX(), pointToCheck.getY(), 0),
      //                                          new Point3D(endPoint.getX(), endPoint.getY(), 0), 0.005, Color.RED);

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch();
   }

}