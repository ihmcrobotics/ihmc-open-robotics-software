package us.ihmc.humanoidBehaviors.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.footstepPlanning.ui.viewers.StartGoalOrientationViewer;
import us.ihmc.footstepPlanning.ui.viewers.StartGoalPositionViewer;
import us.ihmc.humanoidBehaviors.ui.controllers.PatrolUIController;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.humanoidBehaviors.ui.BehaviorUIMessagerAPI.*;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final StartGoalOrientationEditor orientationEditor;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final StartGoalOrientationViewer startGoalOrientationViewer;
   private final JavaFXRobotVisualizer robotVisualizer;

   @FXML private PatrolUIController patrolUIController;

   public BehaviorUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParameters plannerParameters, VisibilityGraphsParameters visibilityGraphsParameters, FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory, RobotContactPointParameters<RobotSide> contactPointParameters, WalkingControllerParameters walkingControllerParameters) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      patrolUIController.attachMessager(messager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      startGoalPositionViewer = new StartGoalPositionViewer(messager, WaypointAPositionEditModeEnabledTopic, WaypointBPositionEditModeEnabledTopic,
                                                            WaypointAPositionTopic, LowLevelGoalPositionTopic, WaypointBPositionTopic);
      startGoalOrientationViewer = new StartGoalOrientationViewer(messager);
      startGoalEditor = new StartGoalPositionEditor(messager, subScene, WaypointAPositionEditModeEnabledTopic, WaypointBPositionEditModeEnabledTopic,
                                                    WaypointAPositionTopic, WaypointBPositionTopic, PlanarRegionDataTopic, SelectedRegionTopic,
                                                    WaypointAOrientationEditModeEnabledTopic, WaypointBOrientationEditModeEnabledTopic);
      orientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene());

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());

      if(fullHumanoidRobotModelFactory == null)
      {
         robotVisualizer = null;
      }
      else
      {
         robotVisualizer = new JavaFXRobotVisualizer(fullHumanoidRobotModelFactory);
         messager.registerTopicListener(RobotConfigurationDataTopic, robotVisualizer::submitNewConfiguration);
         patrolUIController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
      }

      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      Scene mainScene = new Scene(mainPane, 1200, 800);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void setRobotLowLevelMessenger(RobotLowLevelMessenger robotLowLevelMessenger)
   {
      patrolUIController.setRobotLowLevelMessenger(robotLowLevelMessenger);
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      startGoalPositionViewer.stop();
      startGoalOrientationViewer.stop();
      startGoalEditor.stop();
      orientationEditor.stop();

      if(robotVisualizer != null)
         robotVisualizer.stop();
   }

   public static BehaviorUI createMessagerUI(Stage primaryStage, JavaFXMessager messager,
                                             FootstepPlannerParameters plannerParameters,
                                             VisibilityGraphsParameters visibilityGraphsParameters,
                                             FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                                             RobotContactPointParameters<RobotSide> contactPointParameters,
                                             WalkingControllerParameters walkingControllerParameters)
         throws Exception
   {
      return new BehaviorUI(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullHumanoidRobotModelFactory,
                                   contactPointParameters, walkingControllerParameters);
   }
}
