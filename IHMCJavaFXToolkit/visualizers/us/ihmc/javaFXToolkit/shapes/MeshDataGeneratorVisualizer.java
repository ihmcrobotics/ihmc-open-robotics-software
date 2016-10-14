package us.ihmc.javaFXToolkit.shapes;

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
import us.ihmc.graphics3DAdapter.graphics.MeshDataGenerator;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.graphics.JavaFXMeshDataInterpreter;

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

      Material defaultMaterial = new PhongMaterial(Color.CYAN);

      MeshView torus = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.ArcTorus(0.0, 2.0 * Math.PI, 0.3, 0.1, 64)));
      torus.setMaterial(defaultMaterial);
      torus.setTranslateY(1.0);
      rootNode.getChildren().add(torus);

      MeshView cylinder = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cylinder(0.1, 0.3, 64)));
      cylinder.setMaterial(defaultMaterial);
      cylinder.setTranslateY(0.5);
      rootNode.getChildren().add(cylinder);
      
      MeshView cone = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Cone(0.3, 0.1, 64)));
      cone.setMaterial(defaultMaterial);
      cone.setTranslateY(0.0);
      rootNode.getChildren().add(cone);
      
      MeshView sphere = new MeshView(JavaFXMeshDataInterpreter.interpretMeshData(MeshDataGenerator.Sphere(0.2, 32, 32)));
      sphere.setMaterial(defaultMaterial);
      sphere.setTranslateX(0.5);
      rootNode.getChildren().add(sphere);

      primaryStage.setScene(scene);
      primaryStage.show();
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
