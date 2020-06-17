package us.ihmc.humanoidBehaviors.ui;

import javafx.application.Platform;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.*;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.*;
import us.ihmc.humanoidBehaviors.ui.behaviors.DirectRobotUIController;
import us.ihmc.javafx.JavaFXLinuxGUIRecorder;
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

import java.util.HashMap;
import java.util.Map;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private static boolean RECORD_VIDEO = Boolean.parseBoolean(System.getProperty("record.video"));

   private BorderPane mainPane;
   private final Messager behaviorMessager;
   private final Map<String, BehaviorUIInterface> behaviorUIInterfaces = new HashMap<>();

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
      this.behaviorMessager = behaviorMessager;

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

         for (BehaviorUIDefinition uiDefinitionEntry : behaviorUIRegistry.getUIDefinitionEntries())
         {
            BehaviorUIInterface behaviorUIInterface = uiDefinitionEntry.getBehaviorUISupplier().get();
            behaviorUIInterfaces.put(uiDefinitionEntry.getName(), behaviorUIInterface);
            Tab tab = new Tab(uiDefinitionEntry.getName(), JavaFXMissingTools.loadFromFXML(behaviorUIInterface));
            tabPane.getTabs().add(tab);
         }

         View3DFactory view3DFactory = View3DFactory.createSubscene();
         view3DFactory.addCameraController(0.05, 2000.0, true);
         view3DFactory.addWorldCoordinateSystem(0.3);
         view3DFactory.addDefaultLighting();
         SubScene subScene3D = view3DFactory.getSubScene();
         Pane view3DSubSceneWrappedInsidePane = view3DFactory.getSubSceneWrappedInsidePane();

//         AnchorPane sideVisualizationArea = new AnchorPane();
         VBox sideVisualizationArea = new VBox();

         ScrollPane logScrollPane = new ScrollPane();
//         logScrollPane.setLayoutY(500.0);
//         logScrollPane.setPrefWidth(200.0);
//         logScrollPane.setPrefHeight(500.0);
         TextFlow textFlow = new TextFlow();
         textFlow.setTextAlignment(TextAlignment.LEFT);
//         textFlow.setPrefWidth(100.0);
//         TextArea textArea = new TextArea();
//         textArea.setPrefWidth(Region.USE_COMPUTED_SIZE);
//         textArea.setPrefHeight(Region.USE_COMPUTED_SIZE);
         logScrollPane.setContent(textFlow);
//         logScrollPane.setFitToHeight(true);
//         logScrollPane.setFitToWidth(true);
//         logScrollPane.setLayoutY(300.0);
//         logScrollPane.setPrefWidth(300.0);

         behaviorSelector.getItems().add("None");
         behaviorSelector.setValue("None");

         for (BehaviorDefinition behaviorDefinition : behaviorUIRegistry.getDefinitionEntries())
         {
            behaviorSelector.getItems().add(behaviorDefinition.getName());
         }

         for (BehaviorUIInterface behaviorUIInterface : behaviorUIInterfaces.values())
         {
            behaviorUIInterface.init(subScene3D, sideVisualizationArea, ros2Node, behaviorMessager, robotModel);
            view3DFactory.addNodeToView(behaviorUIInterface);
         }

         behaviorSelector.valueProperty().addListener(this::onBehaviorSelection);

         directRobotUIController.init(subScene3D, ros2Node, robotModel);
         view3DFactory.addNodeToView(directRobotUIController);
         view3DFactory.addNodeToView(new JavaFXRemoteRobotVisualizer(robotModel, ros2Node));

         SplitPane mainSplitPane = (SplitPane) mainPane.getCenter();
//         sideVisualizationArea.setPrefWidth(200.0);
//         view3DSubSceneWrappedInsidePane.setPrefWidth(500.0);
         mainSplitPane.getItems().add(view3DSubSceneWrappedInsidePane);
         mainSplitPane.getItems().add(logScrollPane);
//         mainSplitPane.getItems().add(sideVisualizationArea);
         mainSplitPane.setDividerPositions(2.0 / 3.0);

         Stage primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         Scene mainScene = new Scene(mainPane, 1750, 1000);

         primaryStage.setScene(mainScene);
         primaryStage.show();
         primaryStage.toFront();

         if (RECORD_VIDEO)
         {
            JavaFXLinuxGUIRecorder guiRecorder = new JavaFXLinuxGUIRecorder(primaryStage, 24, 0.8f, getClass().getSimpleName());
            guiRecorder.deleteOldLogs(10);
            ThreadTools.scheduleSingleExecution("DelayRecordingStart", guiRecorder::start, 2.0);
            ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 300.0);
            primaryStage.setOnCloseRequest(event -> guiRecorder.stop());
            Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop));
         }

         // do this last for now in case events starts firing early
         String fontName = null;
         for (String installedFontName : Font.getFontNames())
         {
            if (installedFontName.equals("Courier New"))
            {
               fontName = "Courier New";
               break;
            }
            else if (installedFontName.equals("Liberation Mono"))
            {
               fontName = "Liberation Mono";
               break;
            }
         }
         if (fontName == null)
         {
            throw new RuntimeException("No monospaced font found. Please add your font to the code.");
         }
         final String selectedFontName = fontName;
         behaviorMessager.registerTopicListener(BehaviorModule.API.StatusLog, logEntry ->
         {
            Platform.runLater(() ->
            {
               Text text = new Text(logEntry.getRight() + "\n");
               text.setFont(new Font("Courier New", -1));
               text.setFontSmoothingType(FontSmoothingType.LCD);
               switch (logEntry.getLeft())
               {
                  case 100:
                  case 200:
                     text.setFill(Color.RED.brighter());
                     break;
                  case 300:
                     text.setFill(Color.YELLOW.darker());
                     break;
                  case 400:
                     text.setFill(Color.BLACK);
                     break;
                  case 500:
                     text.setFill(Color.CYAN);
                     break;
                  case 600:
                     text.setFill(Color.GREEN);
                     break;
               }
               textFlow.getChildren().add(text);
            });
         });
      });
   }

   private void onBehaviorSelection(ObservableValue<? extends String> observable, String oldValue, String newValue)
   {
      behaviorMessager.submitMessage(BehaviorModule.API.BehaviorSelection, newValue);

      for (String behaviorName : behaviorUIInterfaces.keySet())
      {
         if (newValue.equals(behaviorName))
         {
            behaviorUIInterfaces.get(behaviorName).setEnabled(true);
         }
         else
         {
            behaviorUIInterfaces.get(behaviorName).setEnabled(false);
         }
      }
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
