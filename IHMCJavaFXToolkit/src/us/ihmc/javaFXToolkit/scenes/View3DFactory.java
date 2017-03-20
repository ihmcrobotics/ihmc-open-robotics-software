package us.ihmc.javaFXToolkit.scenes;

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
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;

/**
 * This helper class allows the creation of a JavaFX {@link Scene} or {@link SubScene} setup for 3D rendering.
 * It gathers the common tools that are generally needed to do so, such as a camera controller, lights, world coordinate system.
 * At any time the {@link Scene} or {@link SubScene} can be accessed and customized at will.
 * @author Sylvain Bertrand
 */
public class View3DFactory
{
   /**
    * Available scene options for the factory.
    */
   public enum SceneType
   {
      MAIN_SCENE, SUB_SCENE
   };

   private final Group root = new Group();
   private final Scene scene;
   private final SubScene subScene;

   /**
    * Creates a factory that will create and setup a {@link Scene} with the depth buffer and anti-aliasing defaulted to {@code true} and {@code BALANCED}, respectively.
    * @param width the width of the scene in pixels.
    * @param height the height of the scene in pixels.
    */
   public View3DFactory(double width, double height)
   {
      this(width, height, true);
   }

   /**
    * Creates a factory that will create and setup a {@link Scene} with the anti-aliasing defaulted to BALANCED.
    * @param width the width of the scene in pixels.
    * @param height the height of the scene in pixels.
    * @param depthBuffer the depth buffer flag.
    */
   public View3DFactory(double width, double height, boolean depthBuffer)
   {
      this(width, height, depthBuffer, SceneAntialiasing.BALANCED, SceneType.MAIN_SCENE);
   }

   /**
    * Creates a factory that will create and setup a {@link Scene}.
    * @param width the width of the scene in pixels.
    * @param height the height of the scene in pixels.
    * @param depthBuffer the depth buffer flag.
    * @param antiAliasing the scene anti-aliasing attribute. A value of {@code null} is treated as DISABLED.
    * @param sceneType the type of the scene to create, i.e. {@code MAIN_SCENE} for creating a {@link Scene} or {@code SUB_SCENE} for creating a {@link SubScene}.
    */
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

   /**
    * @return a factory that will create and setup a {@link SubScene} with the depth buffer and anti-aliasing defaulted to {@code true} and {@code BALANCED}, respectively.
    */
   public static View3DFactory createSubscene()
   {
      return createSubscene(true, SceneAntialiasing.BALANCED);
   }

   /**
    * @param depthBuffer the depth buffer flag.
    * @param antiAliasing the scene anti-aliasing attribute. A value of {@code null} is treated as DISABLED.
    * @return a factory that will create and setup a {@link SubScene}.
    */
   public static View3DFactory createSubscene(boolean depthBuffer, SceneAntialiasing antiAliasing)
   {
      return new View3DFactory(-1, -1, depthBuffer, antiAliasing, SceneType.SUB_SCENE);
   }

   /**
    * Add a {@link Color#WHITE} point light to the 3D view at the given coordinate.
    * @param x the light x-coordinate.
    * @param y the light y-coordinate.
    * @param z the light z-coordinate.
    */
   public void addPointLight(double x, double y, double z)
   {
      addPointLight(x, y, z, Color.WHITE);
   }

   /**
    * Add a point light to the 3D view at the given coordinate.
    * @param x the light x-coordinate.
    * @param y the light y-coordinate.
    * @param z the light z-coordinate.
    * @param color the light color.
    */
   public void addPointLight(double x, double y, double z, Color color)
   {
      PointLight light = new PointLight(color);
      light.setTranslateX(x);
      light.setTranslateY(y);
      light.setTranslateZ(z);
      addNodeToView(light);
   }

   /**
    * Add a {@link PerspectiveCamera} with a default controller to the 3D view.
    * @return the camera controller.
    */
   public FocusBasedCameraMouseEventHandler addCameraController()
   {
      return addCameraController(0.05, 50.0, false);
   }

   /**
    * Add a {@link PerspectiveCamera} with a default controller to the 3D view.
    * @param enableShiftClickFocusTranslation whether to use the quick focus translation with Shift + Mouse left click or not.
    * @return the camera controller.
    */
   public FocusBasedCameraMouseEventHandler addCameraController(boolean enableShiftClickFocusTranslation)
   {
      return addCameraController(0.05, 50.0, enableShiftClickFocusTranslation);
   }

   /**
    * Add a {@link PerspectiveCamera} with a default controller to the 3D view.
    * @param nearClip specifies the distance from the eye of the near clipping plane of the {@code Camera} in the eye coordinate space.
    * @param farClip specifies the distance from the eye of the far clipping plane of the {@code Camera} in the eye coordinate space.
    * @param enableShiftClickFocusTranslation whether to use the quick focus translation with Shift + Mouse left click or not.
    * @return the camera controller.
    */
   public FocusBasedCameraMouseEventHandler addCameraController(double nearClip, double farClip, boolean enableShiftClickFocusTranslation)
   {
      PerspectiveCamera camera = new PerspectiveCamera(true);
      camera.setNearClip(nearClip);
      camera.setFarClip(farClip);
      setCamera(camera);

      Vector3D up = new Vector3D(0.0, 0.0, 1.0);
      Vector3D forward = new Vector3D(1.0, 0.0, 0.0);
      ReadOnlyDoubleProperty widthProperty = widthProperty();
      ReadOnlyDoubleProperty heightProperty = heightProperty();
      FocusBasedCameraMouseEventHandler cameraController = new FocusBasedCameraMouseEventHandler(widthProperty, heightProperty, camera, up, forward);
      if (enableShiftClickFocusTranslation)
         cameraController.enableShiftClickFocusTranslation();
      addEventHandler(Event.ANY, cameraController);
      addNodeToView(cameraController.getFocusPointViz());

      if (subScene != null)
         subScene.setOnMouseClicked(event -> subScene.requestFocus());

      return cameraController;
   }

   /**
    * Registers an event handler to the scene being created.
    * The handler is called when the scene receives an {@code Event} of the specified type during the bubbling phase of event delivery.
    * @param <T> the specific event class of the handler
    * @param eventType the type of the events to receive by the handler
    * @param eventHandler the handler to register
    */
   public <T extends Event> void addEventHandler(EventType<T> eventType, EventHandler<? super T> eventHandler)
   {
      if (scene != null)
         scene.addEventHandler(eventType, eventHandler);
      else
         subScene.addEventHandler(eventType, eventHandler);
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

   /**
    * Display the world coordinate system.
    * @param arrowLength length of each axis of the coordinate system.
    */
   public void addWorldCoordinateSystem(double arrowLength)
   {
      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(arrowLength);
      addNodeToView(worldCoordinateSystem);
   }

   /**
    * Add a set of nodes to the 3D view.
    * @param nodes the nodes to display.
    */
   public void addNodesToView(Iterable<? extends Node> nodes)
   {
      nodes.forEach(this::addNodeToView);
   }

   /**
    * Add a node to the 3D view.
    * @param node the node to display.
    */
   public void addNodeToView(Node node)
   {
      root.getChildren().add(node);
   }

   /**
    * If true, this node (together with all its children) is completely transparent to mouse events.
    * When choosing target for mouse event, nodes with {@code mouseTransparent} set to {@code true} and their subtrees won't be taken into account greatly reducing computation load especially when rendering many nodes.
    * @param value whether to make the entire scene mouse transparent or not.
    */
   public void setRootMouseTransparent(boolean value)
   {
      root.setMouseTransparent(value);
   }

   /**
    * @return the root node of the scene in creation.
    */
   public Group getRoot()
   {
      return root;
   }

   /**
    * @return the created scene. Returns null if the factory is creating a sub-scene.
    */
   public Scene getScene()
   {
      return scene;
   }
   
   /**
    * @return the created sub-scene. Returns null if the factory is creating a scene.
    */
   public SubScene getSubScene()
   {
      return subScene;
   }

   /**
    * <p> Only available for sub-scene creation. </p>
    * Wrap the sub-scene in a {@link Pane}, bind its size properties to the Pane, and returns it.
    * It is the preferred option when creating a UI with several elements.
    * @return the the Pane in which the sub-scene is wrapped.
    */
   public Pane getSubSceneWrappedInsidePane()
   {
      Pane pane = new Pane(subScene);
      bindSubSceneSizeToPaneSize(pane);
      return pane;
   }

   /**
    * <p> Only available for sub-scene creation. </p>
    * Adds the created sub-scene to the pane's children and binds the sub-scene size to the pane's size.
    * @param pane the pane the sub-scene is to be attached to.
    */
   public void attachSubSceneTo(Pane pane)
   {
      if (subScene == null)
         return;
      pane.getChildren().add(subScene);
      bindSubSceneSizeToPaneSize(pane);
   }

   /**
    * <p> Only available for sub-scene creation. </p>
    * Binds the sub-scene size to the pane's size.
    * @param pane the pane the sub-scene's size is to be bind to.
    */
   public void bindSubSceneSizeToPaneSize(Pane pane)
   {
      if (subScene == null)
         return;
      subScene.heightProperty().bind(pane.heightProperty());
      subScene.widthProperty().bind(pane.widthProperty());
   }
   
   /**
    * Set background of scene or subscene.
    * @param color
    */
   public void setBackgroundColor(Color color)
   {
      if (scene != null)
      {
         scene.setFill(color);
      }
      else
      {
         subScene.setFill(color);
      }
   }
}
