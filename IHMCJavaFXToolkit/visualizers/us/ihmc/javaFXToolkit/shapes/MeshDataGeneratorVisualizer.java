package us.ihmc.javaFXToolkit.shapes;

import java.util.Random;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.Material;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.graphics3DDescription.MeshDataGenerator;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.random.RandomTools;

public class MeshDataGeneratorVisualizer extends Application
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 600, 400, true);
      scene.setFill(Color.GRAY);
      setupCamera(rootNode, scene);
      
      JavaFXCoordinateSystem coordinateSystem = new JavaFXCoordinateSystem(0.4);
      rootNode.getChildren().add(coordinateSystem);

      Material defaultMaterial = new PhongMaterial(Color.CYAN);

      MeshView torus = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.ArcTorus(0.0, 1.0 * Math.PI, 0.3, 0.1, 64)));
      torus.setMaterial(defaultMaterial);
      torus.setTranslateY(1.5);
      rootNode.getChildren().add(torus);

      MeshView cylinder = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cylinder(0.1, 0.3, 64)));
      cylinder.setMaterial(defaultMaterial);
      cylinder.setTranslateY(0.5);
      rootNode.getChildren().add(cylinder);
      
      MeshView cone = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cone(0.3, 0.1, 64)));
      cone.setMaterial(defaultMaterial);
      cone.setTranslateY(-0.3);
      rootNode.getChildren().add(cone);
      
      MeshView sphere = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Ellipsoid(0.3, 0.1, 0.4, 64, 64)));
      sphere.setMaterial(defaultMaterial);
      sphere.setTranslateX(0.5);
      rootNode.getChildren().add(sphere);

      Point2d[] polygonVertices = createPolygon();
      MeshView extrudedPolygon = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.ExtrudedPolygon(polygonVertices, 0.1)));
      extrudedPolygon.setMaterial(defaultMaterial);
      extrudedPolygon.setTranslateX(2.0);
      extrudedPolygon.setTranslateY(0.5);
      rootNode.getChildren().add(extrudedPolygon);

      MeshView pyramidCube = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.PyramidCube(0.1, 0.2, 0.1, 0.4)));
      pyramidCube.setMaterial(defaultMaterial);
      pyramidCube.setTranslateX(-1.0);
      pyramidCube.setTranslateY(0.0);
      rootNode.getChildren().add(pyramidCube);

      MeshView genTruncatedCone = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.GenTruncatedCone(0.3, 0.2, 0.07, 0.05, 0.1, 64)));
      genTruncatedCone.setMaterial(defaultMaterial);
      genTruncatedCone.setTranslateX(-0.5);
      genTruncatedCone.setTranslateY(0.0);
      rootNode.getChildren().add(genTruncatedCone);

      MeshView hemiEllipsoid = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.HemiEllipsoid(0.3, 0.1, 0.4, 16, 16)));
      hemiEllipsoid.setMaterial(defaultMaterial);
      hemiEllipsoid.setTranslateX(-0.5);
      hemiEllipsoid.setTranslateY(0.5);
      rootNode.getChildren().add(hemiEllipsoid);

      MeshView polygon = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Polygon(polygonVertices)));
//      polygon.setDrawMode(DrawMode.LINE);
      polygon.setMaterial(defaultMaterial);
      polygon.setTranslateX(-0.5);
      polygon.setTranslateY(3.5);
      rootNode.getChildren().add(polygon);

      MeshView cube = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cube(0.01, 0.01, 1.0, false)));
      cube.setMaterial(defaultMaterial);
      cube.setTranslateX(-0.5);
      cube.setTranslateY(-1.0);
      rootNode.getChildren().add(cube);

      MeshView flatRectangle = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.FlatRectangle(-0.1, 0.1, 0.25, 0.01, 0.2)));
      flatRectangle.setMaterial(defaultMaterial);
      flatRectangle.setTranslateX(0.5);
      flatRectangle.setTranslateY(-1.0);
      rootNode.getChildren().add(flatRectangle);

      MeshView wedge = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Wedge(0.4, 0.2, 0.5)));
      wedge.setMaterial(defaultMaterial);
      wedge.setTranslateX(0.5);
      wedge.setTranslateY(1.0);
      rootNode.getChildren().add(wedge);

      MeshView line = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Line(0.0, 0.0, 0.2, 0.0, 0.5, 0.0, 0.01)));
      line.setMaterial(defaultMaterial);
      line.setTranslateX(-0.5);
      line.setTranslateY(1.0);
      rootNode.getChildren().add(line);
      
      MeshView capsule = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Capsule(0.3, 0.1, 0.05, 0.15, 16, 16)));
      capsule.setMaterial(defaultMaterial);
      capsule.setTranslateX(0.0);
      capsule.setTranslateY(0.0);
      rootNode.getChildren().add(capsule);

      MeshView tetrahedron = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Tetrahedron(0.3)));
      tetrahedron.setMaterial(defaultMaterial);
      tetrahedron.setTranslateX(-1.0);
      tetrahedron.setTranslateY(1.0);
      rootNode.getChildren().add(tetrahedron);

      primaryStage.setMaximized(true);
      primaryStage.setScene(scene);
      primaryStage.show();
   }

   private Point2d[] createPolygon()
   {
      int numberOfPoints = 20;

      ConvexPolygon2d polygon = new ConvexPolygon2d();

      Point2d randomVertex = new Point2d();
      Random random = new Random(234523L);

      while (polygon.getNumberOfVertices() < numberOfPoints)
      {
         randomVertex.add(RandomTools.generateRandomPoint2d(random, 0.03, 0.03));
         polygon.addVertex(randomVertex);
         polygon.update();
      }

      Point2d[] vertices = new Point2d[numberOfPoints];
      int reverseIndex = numberOfPoints;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         vertices[i] = polygon.getVertex(--reverseIndex);
      return vertices;
   }

   private void setupCamera(Group root, Scene scene)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(0.05);
      camera.setFarClip(50.0);
      scene.setCamera(camera);

      Vector3d up = new Vector3d(0.0, 0.0, 1.0);
      FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(scene.widthProperty(), scene.heightProperty(), camera, up);
      scene.addEventHandler(Event.ANY, cameraController);
      root.getChildren().add(cameraController.getFocusPointViz());
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
