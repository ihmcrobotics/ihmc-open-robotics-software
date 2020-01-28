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
import us.ihmc.humanoidBehaviors.ui.behaviors.DirectRobotUIController;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.javafx.graphics.LabelGraphic;
import us.ihmc.humanoidBehaviors.ui.tools.JavaFXRemoteRobotVisualizer;
import us.ihmc.humanoidBehaviors.ui.tools.LocalParameterServer;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

import java.util.ArrayList;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private BorderPane mainPane;

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   @FXML private ChoiceBox<String> behaviorSelector;
   @FXML private DirectRobotUIController directRobotUIController;

   public static BehaviorUI createInterprocess(BehaviorUIRegistry behaviorUIRegistry, DRCRobotModel robotModel, String behaviorModuleAddress)
   {
      return new BehaviorUI(behaviorUIRegistry,
                            RemoteBehaviorInterface.createForUI(behaviorUIRegistry, behaviorModuleAddress),
                            robotModel,
                            PubSubImplementation.FAST_RTPS);
   }

   public static BehaviorUI createIntraprocess(BehaviorUIRegistry behaviorUIRegistry, DRCRobotModel robotModel, Messager behaviorSharedMemoryMessager)
   {
      return new BehaviorUI(behaviorUIRegistry, behaviorSharedMemoryMessager, robotModel, PubSubImplementation.INTRAPROCESS);
   }

   private BehaviorUI(BehaviorUIRegistry behaviorUIRegistry, Messager behaviorMessager, DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
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

         ArrayList<BehaviorUIInterface> behaviorUIInterfaces = new ArrayList<>();
         for (BehaviorUIDefinition uiDefinitionEntry : behaviorUIRegistry.getUIDefinitionEntries())
         {
            BehaviorUIInterface behaviorUIInterface = uiDefinitionEntry.getBehaviorUISupplier().get();
            behaviorUIInterfaces.add(behaviorUIInterface);
            Tab tab = new Tab(uiDefinitionEntry.getName(), JavaFXMissingTools.loadFromFXML(behaviorUIInterface));
            tabPane.getTabs().add(tab);
         }

         View3DFactory view3dFactory = View3DFactory.createSubscene();
         view3dFactory.addCameraController(0.05, 2000.0,true);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addDefaultLighting();
         SubScene subScene = view3dFactory.getSubScene();
         Pane subSceneWrappedInsidePane = view3dFactory.getSubSceneWrappedInsidePane();

         behaviorSelector.getItems().add("None");

         for (BehaviorDefinition behaviorDefinition : behaviorUIRegistry.getDefinitionEntries())
         {
            behaviorSelector.getItems().add(behaviorDefinition.getName());
         }
         behaviorSelector.valueProperty().addListener(
               (observable, oldValue, newValue) -> behaviorMessager.submitMessage(BehaviorModule.API.BehaviorSelection, newValue));

         for (BehaviorUIInterface behaviorUIInterface : behaviorUIInterfaces)
         {
            behaviorUIInterface.init(subScene, behaviorMessager, robotModel);
            view3dFactory.addNodeToView(behaviorUIInterface);
         }

         directRobotUIController.init(subScene, ros2Node, robotModel);
         view3dFactory.addNodeToView(directRobotUIController);
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
