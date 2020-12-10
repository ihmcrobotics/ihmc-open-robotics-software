package us.ihmc.humanoidBehaviors.ui;

import javafx.application.Platform;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.*;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import std_msgs.msg.dds.Empty;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.humanoidBehaviors.*;
import us.ihmc.humanoidBehaviors.ui.behaviors.BehaviorDirectRobotUI;
import us.ihmc.humanoidBehaviors.ui.graphics.ConsoleScrollPane;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
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
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
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
   private final ROS2Node ros2Node;
   private final Map<String, BehaviorUIInterface> behaviorUIInterfaces = new HashMap<>();
   private final Map<String, Boolean> enabledUIs = new HashMap<>();
   private JavaFXLinuxGUIRecorder guiRecorder;
   private ArrayList<Runnable> onCloseRequestListeners = new ArrayList<>();
   private LocalParameterServer parameterServer;
   private FocusBasedCameraMouseEventHandler camera;
   private JavaFXRemoteRobotVisualizer robotVisualizer;

   public static volatile Object ACTIVE_EDITOR; // a tool to assist editors in making sure there isn't more than one active

   @FXML private ChoiceBox<String> behaviorSelector;
   @FXML private CheckBox trackRobot;
   @FXML private Button startRecording;
   @FXML private Button stopRecording;
   private BehaviorDirectRobotUI behaviorDirectRobotUI;

   public static BehaviorUI createInterprocess(BehaviorUIRegistry behaviorUIRegistry, DRCRobotModel robotModel, String behaviorModuleAddress)
   {
      return create(behaviorUIRegistry, robotModel, CommunicationMode.INTERPROCESS, CommunicationMode.INTERPROCESS, behaviorModuleAddress, null);
   }

   public static BehaviorUI createIntraprocess(BehaviorUIRegistry behaviorUIRegistry, DRCRobotModel robotModel, Messager behaviorSharedMemoryMessager)
   {
      return create(behaviorUIRegistry, robotModel, CommunicationMode.INTRAPROCESS, CommunicationMode.INTRAPROCESS, null, behaviorSharedMemoryMessager);
   }

   public static BehaviorUI create(BehaviorUIRegistry behaviorUIRegistry,
                                   DRCRobotModel robotModel,
                                   CommunicationMode ros2CommunicationMode,
                                   CommunicationMode messagerCommunicationMode,
                                   String behaviorModuleAddress,
                                   Messager behaviorSharedMemoryMessager)
   {

      if (messagerCommunicationMode == CommunicationMode.INTRAPROCESS && behaviorSharedMemoryMessager == null)
         throw new RuntimeException("Must pass shared Messager for Messager intraprocess mode.");
      else if (messagerCommunicationMode == CommunicationMode.INTERPROCESS && behaviorModuleAddress == null)
         throw new RuntimeException("Must pass address for Messager interprocess mode.");

      Messager messager = messagerCommunicationMode == CommunicationMode.INTRAPROCESS
            ? behaviorSharedMemoryMessager : RemoteBehaviorInterface.createForUI(behaviorUIRegistry, behaviorModuleAddress);
      return new BehaviorUI(behaviorUIRegistry, messager, robotModel, ros2CommunicationMode.getPubSubImplementation());
   }

   public BehaviorUI(BehaviorUIRegistry behaviorUIRegistry, Messager behaviorMessager, DRCRobotModel robotModel, PubSubImplementation pubSubImplementation)
   {
      this.behaviorMessager = behaviorMessager;

      ros2Node = ROS2Tools.createROS2Node(pubSubImplementation, "behavior_ui");

      if (LabelGraphic.TUNING_MODE)
      {
         parameterServer = new LocalParameterServer(getClass(), 16784);
         LabelGraphic.initializeYoVariables(parameterServer.getRegistry());
         parameterServer.start();
      }

      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(() ->
      {
         mainPane = JavaFXMissingTools.loadFromFXML(this);

         AnchorPane mainAnchorPane = new AnchorPane();

         View3DFactory view3DFactory = View3DFactory.createSubscene();
         camera = view3DFactory.addCameraController(0.05, 2000.0, true);
         double isoZoomOut = 7.0;
         camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
         camera.getTranslate().setX(isoZoomOut / 5.0);
         camera.getTranslate().setY(0.0);
         camera.getTranslate().setZ(0.0);
         view3DFactory.addWorldCoordinateSystem(0.3);
         view3DFactory.addDefaultLighting();
         SubScene subScene3D = view3DFactory.getSubScene();
         Pane view3DSubSceneWrappedInsidePane = view3DFactory.getSubSceneWrappedInsidePane();
         StackPane view3DStackPane = new StackPane(view3DSubSceneWrappedInsidePane);
         AnchorPane.setTopAnchor(view3DStackPane, 0.0);
         AnchorPane.setBottomAnchor(view3DStackPane, 0.0);
         AnchorPane.setLeftAnchor(view3DStackPane, 0.0);
         AnchorPane.setRightAnchor(view3DStackPane, 0.0);
         mainAnchorPane.getChildren().add(view3DStackPane);

         VBox sideVisualizationArea = new VBox();

         BorderPane bottom = (BorderPane) mainPane.getBottom();
         TabPane uiTabPane = new TabPane();
         uiTabPane.setTabClosingPolicy(TabPane.TabClosingPolicy.UNAVAILABLE);
         Node uisPane = uiTabPane;

         behaviorDirectRobotUI = new BehaviorDirectRobotUI();
         bottom.setRight(behaviorDirectRobotUI.getDirectRobotAnchorPane());

         for (BehaviorUIDefinition uiDefinitionEntry : behaviorUIRegistry.getUIDefinitionEntries())
         {
            if (uiDefinitionEntry.getBehaviorUISupplier() != null)
            {
               BehaviorUIInterface behaviorUIInterface = uiDefinitionEntry.getBehaviorUISupplier().create(subScene3D,
                                                                                                          sideVisualizationArea,
                                                                                                          ros2Node,
                                                                                                          behaviorMessager,
                                                                                                          robotModel);
               behaviorUIInterfaces.put(uiDefinitionEntry.getName(), behaviorUIInterface);

               if (behaviorUIRegistry.getNumberOfUIs() == 1)
               {
                  uisPane = behaviorUIInterface.getPane();
               }
               else
               {
                  Tab tab = new Tab(uiDefinitionEntry.getName(), behaviorUIInterface.getPane());
                  uiTabPane.getTabs().add(tab);
               }
               view3DFactory.addNodeToView(behaviorUIInterface.get3DGroup());
            }
         }

         bottom.setCenter(uisPane);

         ConsoleScrollPane consoleScrollPane = new ConsoleScrollPane(behaviorMessager, ros2Node);

         stopRecording.setDisable(true);

         behaviorSelector.getItems().add("None");
         behaviorSelector.setValue("None");

         for (BehaviorDefinition behaviorDefinition : behaviorUIRegistry.getDefinitionEntries())
         {
            behaviorSelector.getItems().add(behaviorDefinition.getName());
         }

         behaviorSelector.valueProperty().addListener(this::onBehaviorSelection);

         behaviorDirectRobotUI.init(mainAnchorPane, ros2Node, robotModel);
         view3DFactory.addNodeToView(behaviorDirectRobotUI);
         robotVisualizer = new JavaFXRemoteRobotVisualizer(robotModel, ros2Node);
         robotVisualizer.setTrackRobot(camera, true);
         view3DFactory.addNodeToView(robotVisualizer);

         SplitPane mainSplitPane = (SplitPane) mainPane.getCenter();
         mainSplitPane.getItems().add(mainAnchorPane);
         mainSplitPane.getItems().add(consoleScrollPane);
         mainSplitPane.setDividerPositions(2.0 / 3.0);

         Stage primaryStage = new Stage();
         primaryStage.setTitle("Behavior UI");
         primaryStage.setMaximized(false);
         Scene mainScene = new Scene(mainPane, 1750, 1000);

         primaryStage.setScene(mainScene);
         primaryStage.show();
         primaryStage.toFront();

         primaryStage.setOnCloseRequest(event -> onCloseRequestListeners.forEach(Runnable::run));

         guiRecorder = new JavaFXLinuxGUIRecorder(primaryStage, 24, 0.8f, getClass().getSimpleName());
         onCloseRequestListeners.add(guiRecorder::stop);
         Runtime.getRuntime().addShutdownHook(new Thread(guiRecorder::stop, "GUIRecorderStop"));

         if (RECORD_VIDEO)
         {
            ThreadTools.scheduleSingleExecution("DelayRecordingStart", this::startRecording, 2.0);
            ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 1200.0);
         }

         IHMCROS2Publisher<Empty> shutdownPublisher = ROS2Tools.createPublisher(ros2Node, BehaviorModule.API.SHUTDOWN);
         onCloseRequestListeners.add(() ->
         {
            LogTools.info("Publishing SHUTDOWN on {}", BehaviorModule.API.SHUTDOWN.getName());
            shutdownPublisher.publish(new Empty());
            destroy();
         });

         if (behaviorUIRegistry.getNumberOfUIs() == 1)
         {
            behaviorSelector.valueProperty().setValue(behaviorUIRegistry.getNameOfOnlyUIBehavior());
         }

         // do this last for now in case events starts firing early
         consoleScrollPane.setupAtEnd();
      });
   }

   public void selectBehavior(BehaviorDefinition behaviorDefinition)
   {
      Platform.runLater(() -> behaviorSelector.valueProperty().setValue(behaviorDefinition.getName()));
   }

   @FXML public void trackRobot()
   {
      robotVisualizer.setTrackRobot(camera, trackRobot.isSelected());
   }

   @FXML public void startRecording()
   {
      startRecording.setDisable(true);
      stopRecording.setDisable(false);
      guiRecorder.deleteOldLogs(15);
      ThreadTools.startAThread(() -> guiRecorder.start(), "RecordingStart");
      ThreadTools.scheduleSingleExecution("SafetyStop", guiRecorder::stop, 3600.0);
   }

   @FXML public void stopRecording()
   {
      startRecording.setDisable(false);
      stopRecording.setDisable(true);
      ThreadTools.startAThread(() -> guiRecorder.stop(), "RecordingStop");
   }

   public void addOnCloseRequestListener(Runnable onCloseRequest)
   {
      onCloseRequestListeners.add(onCloseRequest);
   }

   public void closeMessager()
   {
      ExceptionTools.handle(behaviorMessager::closeMessager, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   private void onBehaviorSelection(ObservableValue<? extends String> observable, String oldValue, String newValue)
   {
      behaviorMessager.submitMessage(BehaviorModule.API.BehaviorSelection, newValue);

      for (String behaviorName : behaviorUIInterfaces.keySet())
      {
         boolean enabled = newValue.equals(behaviorName);

         if (enabledUIs.computeIfAbsent(behaviorName, key -> false) != enabled)
         {
            enabledUIs.put(behaviorName, enabled);
            behaviorUIInterfaces.get(behaviorName).setEnabled(enabled);
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

   public void destroy()
   {
      if (robotVisualizer != null)
         robotVisualizer.destroy();
      if (parameterServer != null)
         parameterServer.destroy();
      for (BehaviorUIInterface behaviorUIInterface : behaviorUIInterfaces.values())
      {
         behaviorUIInterface.destroy();
      }
      behaviorDirectRobotUI.destroy();
      ros2Node.destroy();
   }
}
