package us.ihmc.javaFXToolkit.scenes;

import javax.vecmath.Vector3d;

import javafx.beans.property.ReadOnlyDoubleProperty;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.event.EventType;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.PerspectiveCamera;
import javafx.scene.PointLight;
import javafx.scene.Scene;
import javafx.scene.SceneAntialiasing;
import javafx.scene.SubScene;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

public class View3DFactory
{
   public enum SceneType {MAIN_SCENE, SUB_SCENE};

   private final Group root = new Group();
   private final Scene scene;
   private final SubScene subScene;
   
   public View3DFactory(double width, double height)
   {
      this(width, height, true);
   }

   public View3DFactory(double width, double height, boolean depthBuffer)
   {
      this(width, height, depthBuffer, SceneAntialiasing.BALANCED, SceneType.MAIN_SCENE);
   }

   public View3DFactory(double width, double height, boolean depthBuffer, SceneAntialiasing antiAliasing, SceneType sceneType)
   {
      switch (sceneType)
      {
      case MAIN_SCENE:
         scene = new Scene(root, width, height, depthBuffer, antiAliasing);
         scene.setFill(Color.GRAY);
         subScene = null;
         break;
      case SUB_SCENE:
         subScene = new SubScene(root, width, height, depthBuffer, antiAliasing);
         subScene.setFill(Color.GRAY);
         scene = null;
         break;
      default:
         throw new RuntimeException("Unknown sceneType: " + sceneType);
      }
   }

   public static View3DFactory createSubscene()
   {
      return createSubscene(true, SceneAntialiasing.BALANCED);
   }

   public static View3DFactory createSubscene(boolean depthBuffer, SceneAntialiasing antiAliasing)
   {
      return new View3DFactory(-1, -1, depthBuffer, antiAliasing, SceneType.SUB_SCENE);
   }

   public void addPointLight(double x, double y, double z)
   {
      addPointLight(x, y, z, Color.WHITE);
   }

   public void addPointLight(double x, double y, double z, Color color)
   {
      PointLight light = new PointLight(color);
      light.setTranslateX(x);
      light.setTranslateY(y);
      light.setTranslateZ(z);
      addNodeToView(light);
   }

   public void addCameraController()
   {
      addCameraController(0.05, 50.0);
   }

   public void addCameraController(double nearClip, double farClip)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(nearClip);
      camera.setFarClip(farClip);
      setCamera(camera);

      Vector3d up = new Vector3d(0.0, 0.0, 1.0);
      ReadOnlyDoubleProperty widthProperty = widthProperty();
      ReadOnlyDoubleProperty heightProperty = heightProperty();
      FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(widthProperty, heightProperty, camera, up);
      addEventHandler(Event.ANY, cameraController);
      addNodeToView(cameraController.getFocusPointViz());
   }

   private <T extends Event> void addEventHandler(EventType<T> eventType, EventHandler<? super T> eventHandler)
   {
      if (scene != null)
         scene.addEventHandler(eventType, eventHandler);
      else
         subScene.addEventHandler(eventType, eventHandler);
   }

   public void enableRequestFocusOnMouseClicked()
   {
      if (subScene != null)
         subScene.setOnMouseClicked(event -> subScene.requestFocus());
   }

   private void setCamera(PerspectiveCamera camera)
   {
      if (scene != null)
         scene.setCamera(camera);
      else
         subScene.setCamera(camera);
   }

   private ReadOnlyDoubleProperty widthProperty()
   {
      return scene != null ? scene.widthProperty() : subScene.widthProperty();
   }

   private ReadOnlyDoubleProperty heightProperty()
   {
      return scene != null ? scene.heightProperty() : subScene.heightProperty();
   }

   public void addWorldCoordinateSystem(double arrowLength)
   {
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(arrowLength);
      addNodeToView(worldCoordinateSystem);
   }

   public void addNodesToView(Iterable<Node> nodes)
   {
      nodes.forEach(this::addNodeToView);
   }

   public void addNodeToView(Node node)
   {
      root.getChildren().add(node);
   }

   public void setRootMouseTransparent(boolean value)
   {
      root.setMouseTransparent(value);
   }

   public Group getRoot()
   {
      return root;
   }

   public Scene getScene()
   {
      return scene;
   }

   public SubScene getSubScene()
   {
      return subScene;
   }

   public void attachSubSceneTo(Pane pane)
   {
      pane.getChildren().add(subScene);
      subScene.heightProperty().bind(pane.heightProperty());
      subScene.widthProperty().bind(pane.widthProperty());
   }
}
