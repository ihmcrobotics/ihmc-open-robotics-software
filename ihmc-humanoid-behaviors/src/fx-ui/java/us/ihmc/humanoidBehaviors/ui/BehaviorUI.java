package us.ihmc.humanoidBehaviors.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.AmbientLight;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.humanoidBehaviors.ui.controllers.PatrolUIController;
import us.ihmc.humanoidBehaviors.ui.editors.FXUIEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor.API;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.SnappedPositionGraphic;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.humanoidBehaviors.ui.BehaviorUI.API.*;

/**
 * This class constructs a UI for behavior operation.
 */
public class BehaviorUI
{
   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionsGraphic;
   private final SnappedPositionEditor snappedPositionEditor;
   private final SnappedPositionGraphic snappedPositionGraphic;
//   private final StartGoalPositionEditor startGoalEditor;
//   private final StartGoalOrientationEditor orientationEditor;
//   private final StartGoalPositionViewer startGoalPositionViewer;
//   private final StartGoalOrientationViewer startGoalOrientationViewer;
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
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      planarRegionsGraphic = new PlanarRegionsGraphic(messager, PlanarRegionDataTopic);
      snappedPositionEditor = new SnappedPositionEditor(messager, subScene, PlanarRegionDataTopic, ActiveEditor);
      snappedPositionGraphic = new SnappedPositionGraphic(messager, Color.GREEN, SnappedPositionEditor.API.SelectedPosition);
//      startGoalPositionViewer = new StartGoalPositionViewer(messager, WaypointAPositionEditModeEnabledTopic, WaypointBPositionEditModeEnabledTopic,
//                                                            WaypointAPositionTopic, LowLevelGoalPositionTopic, WaypointBPositionTopic);
//      startGoalOrientationViewer = new StartGoalOrientationViewer(messager);
//      startGoalEditor = new StartGoalPositionEditor(messager, subScene, WaypointAPositionEditModeEnabledTopic, WaypointBPositionEditModeEnabledTopic,
//                                                    WaypointAPositionTopic, WaypointBPositionTopic, PlanarRegionDataTopic, SelectedRegionTopic,
//                                                    WaypointAOrientationEditModeEnabledTopic, WaypointBOrientationEditModeEnabledTopic);
//      orientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene());

      view3dFactory.addNodeToView(planarRegionsGraphic.getRoot());
//      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
//      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());

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

      planarRegionsGraphic.start();
      snappedPositionEditor.start();
//      startGoalPositionViewer.start();
//      startGoalOrientationViewer.start();
//      startGoalEditor.start();
//      orientationEditor.start();

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
      planarRegionsGraphic.stop();
      snappedPositionEditor.stop();
//      startGoalPositionViewer.stop();
//      startGoalOrientationViewer.stop();
//      startGoalEditor.stop();
//      orientationEditor.stop();

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

   public static class API
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final MessagerAPIFactory.Category Category = apiFactory.createRootCategory(apiFactory.createCategoryTheme(SnappedPositionEditor.class.getSimpleName()));
      private static final TopicTheme Theme = apiFactory.createTopicTheme("Default");

      public static final Topic<FXUIEditor> ActiveEditor = Category.topic(Theme);

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
