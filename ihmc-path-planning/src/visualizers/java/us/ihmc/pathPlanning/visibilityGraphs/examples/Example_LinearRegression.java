package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.util.ArrayList;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.tools.LinearRegression3D;

public class Example_LinearRegression extends Application
{
   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;
   
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      
      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette );
      
      Point3D point1 = new Point3D(2.328, -0.611, 0.000);
      Point3D point2 = new Point3D(2.287, -0.569, 0.000);
      Point3D point3 = new Point3D(2.278, -0.561, 0.000);
      Point3D point4 = new Point3D(2.278, -0.561, 0.000);
      Point3D point5 = new Point3D(2.283, -0.566, 0.000);
      Point3D point6 = new Point3D(2.452, -0.736, 0.000);
      Point3D point7 = new Point3D(2.693, -0.976, 0.000);
      Point3D point8 = new Point3D(2.937, -1.221, 0.000);
      Point3D point9 = new Point3D(3.070, -1.353, 0.000);
      Point3D point10 = new Point3D(3.087, -1.370, 0.000);
      Point3D point11 = new Point3D(3.096, -1.379, 0.000);
      Point3D point12 = new Point3D(3.100, -1.383, 0.000);
      Point3D point13 = new Point3D(3.096, -1.378, 0.000);
      Point3D point14 = new Point3D(3.089, -1.372, 0.000);
      Point3D point15 = new Point3D(3.067, -1.350, 0.000);
      Point3D point16 = new Point3D(3.046, -1.329, 0.000);
      Point3D point17 = new Point3D(2.416, -0.698, 0.000);

      ArrayList<Point3D> points = new ArrayList<>();
      points.add(point1);
      points.add(point2);
      points.add(point3);
      points.add(point4);
      points.add(point5);
      points.add(point6);
      points.add(point7);
      points.add(point8);
      points.add(point9);
      points.add(point10);
      points.add(point11);
      points.add(point12);
      points.add(point13);
      points.add(point14);
      points.add(point15);
      points.add(point16);
      points.add(point17);

      LinearRegression3D linearRegression = new LinearRegression3D(points);
      Point3D[] extremePoints = linearRegression.getTheTwoPointsFurthestApart();

      linearRegression.calculateRegression();
      double y1 = linearRegression.getYFromX(extremePoints[0].getX());
      double y2 = linearRegression.getYFromX(extremePoints[1].getX());

      Point3D linePoint1 = new Point3D(extremePoints[0].getX(), y1, 0);
      Point3D linePoint2 = new Point3D(extremePoints[1].getX(), y2, 0);
      
      visualizeRawPoints(points);
      visualizeExtremePoints(extremePoints);
      visualizeLineEndPoints(linePoint1, linePoint2);
      
      javaFXMultiColorMeshBuilder.addLine(linePoint1, linePoint2, 0.005, Color.YELLOW);
      
      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   
   private void visualizeLineEndPoints(Point3D point1, Point3D point2)
   {
      javaFXMultiColorMeshBuilder.addSphere(0.02,point1, Color.GREEN);
      javaFXMultiColorMeshBuilder.addSphere(0.02,point2, Color.GREEN);
   }
   
   private void visualizeRawPoints(ArrayList<Point3D> points)
   {
      for (Point3D point : points)
      {
         javaFXMultiColorMeshBuilder.addSphere(0.02,point, Color.BROWN);
      }
   }

   private void visualizeExtremePoints(Point3D[] extremePoints)
   {
      javaFXMultiColorMeshBuilder.addSphere(0.02,extremePoints[0], Color.CYAN);
      javaFXMultiColorMeshBuilder.addSphere(0.02,extremePoints[1], Color.CYAN);
   }

   public static void main(String args[])
   {
      launch();
   }

}
