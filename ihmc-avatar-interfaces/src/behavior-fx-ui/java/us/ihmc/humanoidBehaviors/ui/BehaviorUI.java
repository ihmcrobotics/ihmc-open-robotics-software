package us.ihmc.humanoidBehaviors.ui;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.*;
import us.ihmc.humanoidBehaviors.ui.behaviors.NavigationBehaviorUI;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.humanoidBehaviors.ui.tools.LocalParameterServer;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private static boolean showLivePlanarRegionsGraphic = true;

   private BorderPane mainPane;

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   @FXML private ChoiceBox<String> behaviorSelector;

//   @FXML private PatrolBehaviorUIController patrolBehaviorUIController;
//   @FXML private StepInPlaceBehaviorUIController stepInPlaceBehaviorUIController;
//   @FXML private FancyPosesBehaviorUIController fancyPosesBehaviorUIController;
//   @FXML private ExploreAreaBehaviorUIController exploreAreaBehaviorUIController;
//   @FXML private NavigationBehaviorUIController navigationBehaviorUIController;
//   @FXML private PlannerParametersUIController plannerParametersUIController;
//   @FXML private DirectRobotUIController directRobotUIController;

   public static BehaviorUI createInterprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel, String behaviorModuleAddress)
   {
      return new BehaviorUI(behaviorRegistry,
                            RemoteBehaviorInterface.createForUI(behaviorRegistry, behaviorModuleAddress),
                            robotModel,
                            PubSubImplementation.FAST_RTPS);
   }

   public static BehaviorUI createIntraprocess(BehaviorRegistry behaviorRegistry, DRCRobotModel robotModel, Messager behaviorSharedMemoryMessager)
   {
      return new BehaviorUI(behaviorRegistry, behaviorSharedMemoryMessager, robotModel, PubSubImplementation.INTRAPROCESS);
   }

   private BehaviorUI(BehaviorRegistry behaviorRegistry, Messager behaviorMessager, DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      Ros2Node ros2Node = ROS2Tools.createRos2Node(pubSubImplementation, "behavior_ui");

      if (LabelGraphic.TUNING_MODE)
      {
         LocalParameterServer parameterServer = new LocalParameterServer(getClass(), 16784);
         LabelGraphic.initializeYoVariables(parameterServer.getRegistry());
         parameterServer.start();
      }

      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(() ->
      {
         mainPane = JavaFXMissingTools.loadFromFXML(this);

         BorderPane bottom = (BorderPane) mainPane.getBottom();
         TabPane tabPane = (TabPane) bottom.getCenter();
         LogTools.info("TAB PANEEE {}", tabPane);

         Tab tab = new Tab("Nav", JavaFXMissingTools.loadFromFXML(new NavigationBehaviorUI()));
         tabPane.getTabs().add(tab);

         View3DFactory view3dFactory = View3DFactory.createSubscene();
         view3dFactory.addCameraController(0.05, 2000.0,true);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addDefaultLighting();
         SubScene subScene = view3dFactory.getSubScene();
         Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

         behaviorSelector.getItems().add("NONE");

         for (BehaviorStatics behaviorStatics : behaviorRegistry.getEntries())
         {
            behaviorSelector.getItems().add(behaviorStatics.getName());
         }
         behaviorSelector.valueProperty().addListener(
               (observable, oldValue, newValue) -> behaviorMessager.submitMessage(BehaviorModule.API.BehaviorSelection, newValue));

//         stepInPlaceBehaviorUIController.init(behaviorMessager);
//         fancyPosesBehaviorUIController.init(behaviorMessager);
//         exploreAreaBehaviorUIController.init(subScene, behaviorMessager, robotModel);
//         navigationBehaviorUIController.init(behaviorMessager);
//         patrolBehaviorUIController.init(subScene, behaviorMessager, robotModel);
//         plannerParametersUIController.init(behaviorMessager, robotModel);
//         directRobotUIController.init(ros2Node, robotModel);

//         view3dFactory.addNodeToView(patrolBehaviorUIController);
//         view3dFactory.addNodeToView(exploreAreaBehaviorUIController);

         if (showLivePlanarRegionsGraphic)
         {
            view3dFactory.addNodeToView(new LivePlanarRegionsGraphic(ros2Node, false));
         }

         view3dFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));

         mainPane.setCenter(subSceneWrappedInsidePane);
         Stage primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         Scene mainScene = new Scene(mainPane, 1554, 1000);

         primaryStage.setScene(mainScene);
         primaryStage.show();
         primaryStage.toFront();
      });
   }

   public static void claimEditing(Object claimingEditor)
   {
      if (BehaviorUI.ACTIVE_EDITOR != null)
      {
         throw new RuntimeException("Only one editor may be active at a time.");
      }
      else
      {
         BehaviorUI.ACTIVE_EDITOR = claimingEditor;
         LogTools.debug("editor activated: {}", claimingEditor.getClass().getSimpleName());
      }
   }
}
