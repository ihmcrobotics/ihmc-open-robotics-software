package us.ihmc.humanoidBehaviors.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.BehaviorTeleop;
import us.ihmc.humanoidBehaviors.ui.behaviors.PatrolBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.behaviors.StepInPlaceBehaviorUIController;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIEditor;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.model.interfaces.FXUIEditableGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   // Editors
   public static SnappedPositionEditor SNAPPED_POSITION_EDITOR;
   public static OrientationYawEditor ORIENTATION_EDITOR;

   private final PlanarRegionsGraphic planarRegionsGraphic;
   private final JavaFXRemoteRobotVisualizer robotVisualizer;

   @FXML private StepInPlaceBehaviorUIController stepInPlaceBehaviorUIController;
   @FXML private PatrolBehaviorUIController patrolBehaviorUIController;

   public BehaviorUI(Stage primaryStage,
                     BehaviorTeleop teleop,
                     DRCRobotModel robotModel,
                     PubSubImplementation pubSubImplementation) throws Exception
   {
      this.primaryStage = primaryStage;

      messager = new SharedMemoryJavaFXMessager(BehaviorUI.API.create());
      messager.startMessager();
      
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_ui");

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      {
         /** TODO: Replace with View3DFactory.addDefaultLighting() when javafx-toolkit 0.12.8+ is released */
         double ambientValue = 0.7;
         double pointValue = 0.2;
         double pointDistance = 1000.0;
         Color ambientColor = Color.color(ambientValue, ambientValue, ambientValue);
         view3dFactory.addNodeToView(new AmbientLight(ambientColor));
         Color indoorColor = Color.color(pointValue, pointValue, pointValue);
         view3dFactory.addPointLight(pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(-pointDistance, -pointDistance, pointDistance, indoorColor);
         view3dFactory.addPointLight(pointDistance, -pointDistance, pointDistance, indoorColor);
      }
      SubScene subScene = view3dFactory.getSubScene();
      Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

      stepInPlaceBehaviorUIController.init(teleop);
      patrolBehaviorUIController.init(messager, subScene);

      planarRegionsGraphic = new PlanarRegionsGraphic();
      SNAPPED_POSITION_EDITOR = new SnappedPositionEditor(messager, subScene);
      ORIENTATION_EDITOR = new OrientationYawEditor(messager, subScene);

      view3dFactory.addNodeToView(planarRegionsGraphic.getRoot());
      view3dFactory.addNodeToView(patrolBehaviorUIController.getRoot());

      robotVisualizer = new JavaFXRemoteRobotVisualizer(robotModel, ros2Node);
      patrolBehaviorUIController.setFullRobotModel(robotVisualizer.getFullRobotModel());
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());
      robotVisualizer.start();

      SNAPPED_POSITION_EDITOR.start();
      ORIENTATION_EDITOR.start();

      mainPane.setCenter(subSceneWrappedInsidePane);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1200, 800);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      SNAPPED_POSITION_EDITOR.stop();
      ORIENTATION_EDITOR.stop();

      if(robotVisualizer != null)
         robotVisualizer.stop();

      ExceptionTools.handle(() -> messager.closeMessager(), DefaultExceptionHandler.RUNTIME_EXCEPTION);
   }

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category Root = apiFactory.createRootCategory("Behavior");
      private static final CategoryTheme UI = apiFactory.createCategoryTheme("UI");

      public static final Topic<FXUIEditor> ActiveEditor = Root.child(UI).topic(apiFactory.createTypedTopicTheme("ActiveEditor"));
      public static final Topic<FXUIEditableGraphic> SelectedGraphic = Root.child(UI).topic(apiFactory.createTypedTopicTheme("SelectedGraphic"));
      public static final Topic<FXUIStateMachine> ActiveStateMachine = Root.child(UI).topic(apiFactory.createTypedTopicTheme("ActiveStateMachine"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
