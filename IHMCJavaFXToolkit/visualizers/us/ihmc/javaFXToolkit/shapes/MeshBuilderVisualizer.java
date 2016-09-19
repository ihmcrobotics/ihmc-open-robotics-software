package us.ihmc.javaFXToolkit.shapes;

import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import javafx.application.Application;
import javafx.event.Event;
import javafx.scene.Group;
import javafx.scene.PerspectiveCamera;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.robotics.random.RandomTools;

public class MeshBuilderVisualizer extends Application
{
   private enum MeshToDisplay {BOX, LINE}
   private static final MeshToDisplay MESH_TO_DISPLAY = MeshToDisplay.LINE;

   public MeshBuilderVisualizer()
   {
      // TODO Auto-generated constructor stub
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      Group rootNode = new Group();
      Scene scene = new Scene(rootNode, 600, 400, true);
      scene.setFill(Color.GRAY);
      setupCamera(rootNode, scene);

      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      rootNode.getChildren().add(worldCoordinateSystem);

      MeshBuilder meshBuilder = new MeshBuilder();
      switch (MESH_TO_DISPLAY)
      {
      case BOX:
         addRandomBoxes(meshBuilder);
         break;
      case LINE:
         addLines(meshBuilder);
      default:
         break;
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      PhongMaterial material = new PhongMaterial();
      material.setDiffuseColor(Color.CYAN);
      material.setSpecularColor(Color.CYAN.brighter());
      meshView.setMaterial(material);
      rootNode.getChildren().add(meshView);

      primaryStage.setScene(scene);
      primaryStage.show();
   }

   public void addRandomBoxes(MeshBuilder meshBuilder)
   {
      int count = 0;
      for (float x = -5.0f; x <= 5.0f; x += 0.055f)
      {
         for (float y = -2.0f; y <= 2.0f; y += 0.055f)
         {
            meshBuilder.addBoxMesh(0.05f, 0.05f, 0.05f, new Vector3f(x, y, RandomTools.generateRandomFloatInRange(new Random(), -2.0f, 2.0f)));
            count++;
         }
      }
      System.out.println("Number of boxes: " + count);
   }

   private void addLines(MeshBuilder meshBuilder)
   {
      Point3d start = new Point3d(0.3, 0.0, -0.);
      Point3d end = new Point3d(0.0, 0.3, 0.0);
      double lineWidth = 0.01;
      meshBuilder.addLineMesh(start, end, lineWidth);
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
      Application.launch(args);
   }
}
