package us.ihmc.pathPlanning.visibilityGraphs.examples;

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
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_IsPointInsideConcavePolygon extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();

   double extrusionDistance = 0.60;

   public Example_IsPointInsideConcavePolygon()
   {
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      ArrayList<Point2D> polygon = new ArrayList<>();
      polygon.add(new Point2D(0, 0));
      polygon.add(new Point2D(1, 0));
      polygon.add(new Point2D(1, 1));
      polygon.add(new Point2D(0, 1));
      polygon.add(new Point2D(0, 0.25));
      polygon.add(new Point2D(-1, 0.25));
      polygon.add(new Point2D(-1, 1));
      polygon.add(new Point2D(-2, 1));
      polygon.add(new Point2D(-2, 0));
      polygon.add(new Point2D(0, 0));

      Point2D[] points = polygon.toArray(new Point2D[polygon.size()]);

      Point2D centroid = EuclidGeometryTools.averagePoint2Ds(polygon);
      Point2D pointToCheck = new Point2D(-0.5, 0.5);

      Vector2D directionToCentroid = new Vector2D(centroid.getX() - pointToCheck.getX(), centroid.getY() - pointToCheck.getY());
      directionToCentroid.normalize();
      directionToCentroid.scale(10);

      Point2D endPoint = new Point2D(pointToCheck.getX() + directionToCentroid.getX(), pointToCheck.getY() + directionToCentroid.getY());

      if (PlanarRegionTools.isPointInsidePolygon(points, pointToCheck))
      {
         System.out.println("Is inside the polygon");
      }
      else
      {
         System.out.println("Is outside the polygon");
      }

      javaFXMultiColorMeshBuilder.addLine(new Point3D(pointToCheck.getX(), pointToCheck.getY(), 0), new Point3D(endPoint.getX(), endPoint.getY(), 0), 0.005,
                                          Color.RED);

      javaFXMultiColorMeshBuilder.addSphere(0.1f, new Point3D(pointToCheck.getX(), pointToCheck.getY(), 0), Color.AQUAMARINE);

      for (int i = 1; i < polygon.size(); i++)
      {
         javaFXMultiColorMeshBuilder.addLine(new Point3D(polygon.get(i - 1).getX(), polygon.get(i - 1).getY(), 0),
                                             new Point3D(polygon.get(i).getX(), polygon.get(i).getY(), 0), 0.005, Color.BLACK);
      }

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