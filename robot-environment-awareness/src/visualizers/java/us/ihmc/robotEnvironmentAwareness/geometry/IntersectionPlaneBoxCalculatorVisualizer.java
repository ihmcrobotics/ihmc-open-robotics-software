package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.Box;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;

public class IntersectionPlaneBoxCalculatorVisualizer extends Application
{
   private final TextureColorPalette1D colorPalette = new TextureColorPalette1D();
   private final JavaFXMultiColorMeshBuilder colorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);
   private final IntersectionPlaneBoxCalculator calculator = new IntersectionPlaneBoxCalculator();
   private final Box box;
//   private final Cylinder normalCylinder;

   public IntersectionPlaneBoxCalculatorVisualizer()
   {
      double lx = 0.1;
      double ly = 0.1;
      double lz = 0.1;

      Point3D cubeCenter = new Point3D(-0.25, -0.45, -0.05);
      Point3D pointOnPlane = new Point3D(-0.2242894023656845, -0.4647734463214874, -0.0023258039727807045);
      Vector3D planeNormal = new Vector3D(0.20791170661191224, 1.503689309739766E-8, 0.9781475973766547);

//
//      pointOnPlane.sub(cubeCenter);
//      cubeCenter.set(0.0, 0.0, 0.0);

      calculator.setBox(lx, ly, lz, cubeCenter);
      calculator.setPlane(pointOnPlane, planeNormal);

      colorPalette.setHueBased(0.9, 0.8);

      List<Point3D> intersections = calculator.computeIntersections();
      System.out.println(intersections);
      
      colorMeshBuilder.addPolyon(intersections, Color.DARKCYAN);
      for (int index = 0; index < intersections.size(); index++)
         colorMeshBuilder.addCube(0.01, intersections.get(index), Color.FIREBRICK);
      colorMeshBuilder.addCube(0.02, pointOnPlane, Color.SLATEGREY);
      box = new Box(lx, ly, lz);
      box.setTranslateX(cubeCenter.getX());
      box.setTranslateY(cubeCenter.getY());
      box.setTranslateZ(cubeCenter.getZ());

//      normalCylinder = new Cylinder(0.01, 0.3);
//      normalCylinder.setTranslateX(pointOnPlane.getX());
//      normalCylinder.setTranslateY(pointOnPlane.getY() + 0.5 * normalCylinder.getHeight());
//      normalCylinder.setTranslateZ(pointOnPlane.getZ());
//      AxisAngle4d axisAngle = new AxisAngle4d();
//      GeometryTools.getRotationBasedOnNormal(axisAngle, planeNormal, new Vector3d(0.0, 1.0, 0.0));
//      Affine affine = new Affine();
//      JavaFXTools.convertAxisAngleToAffine(axisAngle, affine);
//      normalCylinder.getTransforms().add(affine);
//      
      for (int i = 0; i < intersections.size(); i++)
      {

         Point3D intersection = intersections.get(i);
         Vector3D v0 = new Vector3D();
         Vector3D v1 = new Vector3D();
         Vector3D v3 = new Vector3D();
         Point3D nextIntersection = intersections.get((i + 1) % intersections.size());
         Point3D previousIntersection = intersections.get(i == 0 ? intersections.size() - 1 : i - 1);
         v0.sub(intersection, nextIntersection);
         v1.sub(intersection, previousIntersection);
         v3.cross(v0, v1);
         System.out.println(v3.dot(planeNormal) < 0.0);
      }
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(800, 600);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);


      MeshView meshView = new MeshView();
      meshView.setMesh(colorMeshBuilder.generateMesh());
      meshView.setMaterial(colorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      PhongMaterial material = new PhongMaterial(Color.DARKCYAN);
//      normalCylinder.setMaterial(material);
//      rootNode.getChildren().add(normalCylinder);
      material = new PhongMaterial();
      material.setDiffuseColor(new Color(1.0, 1.0, 0.0, 0.0));
      box.setMaterial(material);
      view3dFactory.addNodeToView(box);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
