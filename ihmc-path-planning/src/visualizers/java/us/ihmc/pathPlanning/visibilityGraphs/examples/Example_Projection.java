package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.util.ArrayList;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.robotics.geometry.PlanarRegion;

public class Example_Projection extends Application
{
   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      
      Point3D point1 = new Point3D(3.513,  0.351,  0.003);
      Point3D point2 = new Point3D(3.525,  0.873,  0.006);
      Point3D point3 = new Point3D(3.981,  0.977,  0.013);
      
      ArrayList<Point3D> points = new ArrayList<>();
      points.add(point1);
      points.add(point2);
      points.add(point3);
      
      Point3D ptToPorject = new Point3D(3.7,0.9,1);
      javaFXMultiColorMeshBuilder.addSphere(0.02, ptToPorject, Color.RED);

      for (Point3D pt : points)
      {
         javaFXMultiColorMeshBuilder.addSphere(0.02, pt, Color.GREEN);
      }
      
      Vector3D normal = EuclidGeometryTools.normal3DFromThreePoint3Ds(point1, point2, point3);

      Point3D projectedPoint = new Point3D();
      if (!EuclidGeometryTools.orthogonalProjectionOnPlane3D(ptToPorject, point1, normal, projectedPoint))
      {
         projectedPoint = null;
      }
      
      javaFXMultiColorMeshBuilder.addSphere(0.02, projectedPoint, Color.YELLOW);


      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }
   

   private void visualizeLineEndPoints(Point3D point1, Point3D point2)
   {
      javaFXMultiColorMeshBuilder.addSphere(0.02, point1, Color.GREEN);
      javaFXMultiColorMeshBuilder.addSphere(0.02, point2, Color.GREEN);
   }

   private void visualizeRawPoints(ArrayList<Point3D> points)
   {
      for (Point3D point : points)
      {
         javaFXMultiColorMeshBuilder.addSphere(0.02, point, Color.BROWN);
      }
   }

   private void visualizeExtremePoints(Point3D[] extremePoints)
   {
      javaFXMultiColorMeshBuilder.addSphere(0.02, extremePoints[0], Color.CYAN);
      javaFXMultiColorMeshBuilder.addSphere(0.02, extremePoints[1], Color.CYAN);
   }

   public static void main(String args[])
   {
      launch();
   }

}
