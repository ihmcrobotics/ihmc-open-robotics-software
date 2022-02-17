package us.ihmc.footstepPlanning.ui;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import controller_msgs.msg.dds.HeightMapMessage;
import controller_msgs.msg.dds.REAStateRequestMessage;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.ui.components.*;
import us.ihmc.footstepPlanning.ui.controllers.*;
import us.ihmc.footstepPlanning.ui.viewers.*;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javafx.JavaFXRobotVisualizer;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.OcTreeViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

/**
 * User interface for {@link us.ihmc.footstepPlanning.FootstepPlanningModule}. - Compute footstep
 * plans online given live robot and perception data, or offline given a {@link DataSetName} or
 * {@link us.ihmc.footstepPlanning.log.FootstepPlannerLog} - Tuning tabs for body path, footstep,
 * and swing parameters - Log visualization - IK to set chest, arm and head poses - Visualize REA
 * data
 */
public class FootstepPlannerUI
{
   private static final boolean ENABLE_HEIGHT_MAP_VIZ = true;
   private static final boolean SETUP_HEIGHT_MAP_NAV = false;

   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final OcTreeViewer ocTreeViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final GoalOrientationViewer goalOrientationViewer;
   private final FootstepPathMeshViewer pathViewer;
   private final SwingPlanMeshViewer postProcessingViewer;
   private final GoalOrientationEditor orientationEditor;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
   private final JavaFXRobotVisualizer robotVisualizer;
   private final FootstepPlannerLogRenderer footstepPlannerLogRenderer;
   private final BodyPathLogRenderer bodyPathLogRenderer;
   private final ManualFootstepAdjustmentListener manualFootstepAdjustmentListener;
   private final HeightMapNavigationUpdater heightMapNavigationUpdater;
   private final RobotIKVisualizer robotIKVisualizer;
   private final HeightMapVisualizer heightMapVisualizer = new HeightMapVisualizer();

   private final List<Runnable> shutdownHooks = new ArrayList<>();

   // Menu and side bars
   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;
   @FXML
   private FootstepPlannerStatusBarController footstepPlannerStatusBarController;
   @FXML
   private FootstepPlannerTestDashboardController footstepPlannerTestDashboardController;

   // Tabs
   @FXML
   private MainTabController mainTabController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;
   @FXML
   private SwingPlannerParametersUIController swingPlannerParametersUIController;
   @FXML
   private FootstepPlannerLogVisualizerController footstepPlannerLogVisualizerController;
   @FXML
   private BodyPathLogVisualizerController bodyPathLogVisualizerController;
   @FXML
   private RobotOperationTabController robotOperationTabController;
   @FXML
   private VisualizationController visibilityGraphsUIController;

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, boolean showTestDashboard, SideDependentList<List<Point2D>> defaultContactPoints)
         throws Exception
   {
      this(primaryStage,
           messager,
           new DefaultVisibilityGraphParameters(),
           new DefaultFootstepPlannerParameters(),
           new DefaultSwingPlannerParameters(),
           null,
           null,
           showTestDashboard,
           defaultContactPoints);
   }

   public FootstepPlannerUI(Stage primaryStage,
                            JavaFXMessager messager,
                            VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            FootstepPlannerParametersBasics plannerParameters,
                            SwingPlannerParametersBasics swingPlannerParameters,
                            FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                            WalkingControllerParameters walkingControllerParameters,
                            boolean showTestDashboard,
                            SideDependentList<List<Point2D>> defaultContactPoints)
         throws Exception
   {
      this(primaryStage,
           messager,
           visibilityGraphsParameters,
           plannerParameters,
           swingPlannerParameters,
           fullHumanoidRobotModelFactory,
           null,
           null,
           walkingControllerParameters,
           null,
           showTestDashboard,
           defaultContactPoints,
           null);
   }

   public FootstepPlannerUI(Stage primaryStage,
                            JavaFXMessager messager,
                            VisibilityGraphsParametersBasics visibilityGraphsParameters,
                            FootstepPlannerParametersBasics plannerParameters,
                            SwingPlannerParametersBasics swingPlannerParameters,
                            FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                            FullHumanoidRobotModelFactory previewModelFactory,
                            HumanoidJointNameMap jointMap,
                            WalkingControllerParameters walkingControllerParameters,
                            UIAuxiliaryRobotData auxiliaryRobotData,
                            boolean showTestDashboard,
                            SideDependentList<List<Point2D>> defaultContactPoints,
                            CollisionBoxProvider collisionBoxProvider)
         throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();
      SplitPane splitPane = (SplitPane) mainPane.getCenter();
      BorderPane centerBorderPane = (BorderPane) splitPane.getItems().get(0);
      Node testDashboard = centerBorderPane.getLeft();

      if (!showTestDashboard)
      {
         centerBorderPane.getChildren().remove(testDashboard);
      }

      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);
      footstepPlannerParametersUIController.setPlannerParameters(plannerParameters);
      swingPlannerParametersUIController.setParameters(swingPlannerParameters);

      mainTabController.attachMessager(messager);
      footstepPlannerStatusBarController.attachMessager(messager);
      footstepPlannerTestDashboardController.attachMessager(messager);
      footstepPlannerMenuUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      footstepPlannerParametersUIController.attachMessager(messager);
      bodyPathLogVisualizerController.attachMessager(messager);
      swingPlannerParametersUIController.attachMessager(messager);
      footstepPlannerLogVisualizerController.attachMessager(messager);
      visibilityGraphsUIController.attachMessager(messager);
      robotOperationTabController.attachMessager(messager);

      if (ENABLE_HEIGHT_MAP_VIZ)
      {
         messager.registerTopicListener(HeightMapData, heightMapVisualizer::update);
      }

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      mainTabController.bindControls();
      footstepPlannerStatusBarController.bindControls();
      bodyPathLogVisualizerController.bindControls();
      footstepPlannerTestDashboardController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      footstepPlannerLogVisualizerController.bindControls();
      swingPlannerParametersUIController.bindControls();
      visibilityGraphsUIController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addDefaultLighting();

      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      worldCoordinateSystem.setMouseTransparent(true);
      view3dFactory.addNodeToView(worldCoordinateSystem);
      messager.registerTopicListener(ShowCoordinateSystem, worldCoordinateSystem::setVisible);

      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionData, ShowPlanarRegions);
      this.ocTreeViewer = new OcTreeViewer();
      this.startGoalPositionViewer = new StartGoalPositionViewer(messager, null, GoalPositionEditModeEnabled, null, LowLevelGoalPosition, GoalMidFootPosition);
      this.goalOrientationViewer = new GoalOrientationViewer(messager);
      this.goalOrientationViewer.setPlannerParameters(plannerParameters);
      this.startGoalEditor = new StartGoalPositionEditor(messager,
                                                         subScene,
                                                         null,
                                                         GoalPositionEditModeEnabled,
                                                         null,
                                                         GoalMidFootPosition,
                                                         PlanarRegionData,
                                                         SelectedRegion,
                                                         null,
                                                         GoalOrientationEditModeEnabled);
      this.orientationEditor = new GoalOrientationEditor(messager, view3dFactory.getSubScene());
      this.pathViewer = new FootstepPathMeshViewer(messager);
      this.postProcessingViewer = new SwingPlanMeshViewer(messager);
      this.bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      this.visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
      this.footstepPlannerLogRenderer = new FootstepPlannerLogRenderer(defaultContactPoints, messager);
      this.bodyPathLogRenderer = new BodyPathLogRenderer(messager);
      new UIFootstepPlanManager(messager);
      this.manualFootstepAdjustmentListener = new ManualFootstepAdjustmentListener(messager, view3dFactory.getSubScene());
      this.robotOperationTabController.setAuxiliaryRobotData(auxiliaryRobotData);

      startGoalPositionViewer.setShowStartGoalTopics(ShowStart, ShowGoal, ShowGoal);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(ocTreeViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(goalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathViewer.getRoot());
      view3dFactory.addNodeToView(postProcessingViewer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(visibilityGraphsRenderer.getRoot());
      view3dFactory.addNodeToView(footstepPlannerLogRenderer.getRoot());
      view3dFactory.addNodeToView(heightMapVisualizer.getRoot());
      view3dFactory.addNodeToView(bodyPathLogRenderer.getRoot());

      if (fullHumanoidRobotModelFactory == null)
      {
         robotVisualizer = null;
      }
      else
      {
         robotVisualizer = new JavaFXRobotVisualizer(fullHumanoidRobotModelFactory);
         messager.registerTopicListener(RobotConfigurationData, robotVisualizer::submitNewConfiguration);

         mainTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         robotOperationTabController.setFullRobotModel(robotVisualizer.getFullRobotModel(), fullHumanoidRobotModelFactory);
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
         messager.registerTopicListener(ShowRobot, show -> robotVisualizer.getRootNode().setVisible(show));
      }

//      if (previewModelFactory == null)
//      {
         robotIKVisualizer = null;
//      }
//      else
//      {
//         robotIKVisualizer = new RobotIKVisualizer(previewModelFactory, jointMap, messager);
//         messager.registerTopicListener(RobotConfigurationData, robotIKVisualizer::submitNewConfiguration);
//         view3dFactory.addNodeToView(robotIKVisualizer.getRootNode());
//      }

      if (walkingControllerParameters != null)
      {
         mainTabController.setDefaultTiming(walkingControllerParameters.getDefaultSwingTime(), walkingControllerParameters.getDefaultTransferTime());
      }

      if (defaultContactPoints != null)
      {
         mainTabController.setContactPointParameters(defaultContactPoints);
         pathViewer.setDefaultContactPoints(defaultContactPoints);
         goalOrientationViewer.setDefaultContactPoints(defaultContactPoints);
         footstepPlannerLogVisualizerController.setContactPointParameters(defaultContactPoints);
      }

      messager.registerTopicListener(ShowOcTree, ocTreeViewer::setEnabled);
      messager.registerTopicListener(OcTreeData, ocTreeViewer::submitOcTreeData);

      planarRegionViewer.start();
      ocTreeViewer.start();
      startGoalPositionViewer.start();
      goalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathViewer.start();
      postProcessingViewer.start();
      bodyPathMeshViewer.start();
      visibilityGraphsRenderer.start();
      footstepPlannerLogRenderer.start();
      bodyPathLogRenderer.start();
      manualFootstepAdjustmentListener.start();
      new FootPoseFromMidFootUpdater(messager).start();
      new FootstepCompletionListener(messager).start();

      if (robotVisualizer != null && SETUP_HEIGHT_MAP_NAV)
      {
         heightMapNavigationUpdater = new HeightMapNavigationUpdater(messager, plannerParameters, walkingControllerParameters, defaultContactPoints, robotVisualizer.getFullRobotModel(), collisionBoxProvider);
         heightMapNavigationUpdater.start();
      }
      else
      {
         heightMapNavigationUpdater = null;
      }

      heightMapVisualizer.start();

      messager.registerTopicListener(HeightMapData, data -> planarRegionViewer.clear());
      messager.registerTopicListener(PlanarRegionData, data -> heightMapVisualizer.clear());
      messager.registerTopicListener(ShowHeightMap, show -> heightMapVisualizer.getRoot().setVisible(show));

      if (auxiliaryRobotData != null)
      {
         setupDataSetLoadBingings(auxiliaryRobotData);
      }

      centerBorderPane.setCenter(subScene);

      primaryStage.setTitle(getClass().getSimpleName());
      //      primaryStage.setMaximized(true);

      primaryStage.maximizedProperty().addListener((observable, oldValue, newValue) ->
      {
         splitPane.setDividerPositions(0.7);
      });
      splitPane.getDividers().get(0).positionProperty().addListener((observable, oldValue, newValue) ->
      {
         if (newValue.doubleValue() > 0.75)
         {
            splitPane.getDividers().get(0).positionProperty().setValue(0.75);
         }
      });
      Scene mainScene = new Scene(mainPane);

      mainScene.getStylesheets().add("us/ihmc/footstepPlanning/ui/FootstepPlannerUI.css");

      primaryStage.setScene(mainScene);
      primaryStage.setWidth(1910);
      primaryStage.setHeight(1070);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void setupDataSetLoadBingings(UIAuxiliaryRobotData auxiliaryRobotData)
   {
      Consumer<DataSetName> dataSetLoader = dataSetName ->
      {
         DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
         messager.submitMessage(PlanarRegionData, dataSet.getPlanarRegionsList());

         RigidBodyTransform robotRootJoint = new RigidBodyTransform();
         if (dataSet.hasPlannerInput())
         {
            robotRootJoint.getTranslation().set(dataSet.getPlannerInput().getStartPosition());
            robotRootJoint.getRotation().setYawPitchRoll(dataSet.getPlannerInput().getStartYaw(), 0.0, 0.0);
            robotRootJoint.getTranslation().sub(auxiliaryRobotData.getRootJointToMidFootOffset());
            robotVisualizer.submitNewConfiguration(robotRootJoint, auxiliaryRobotData.getDefaultJointAngleMap());

            if (robotIKVisualizer != null)
               robotIKVisualizer.submitNewConfiguration(robotRootJoint, auxiliaryRobotData.getDefaultJointAngleMap());

            messager.submitMessage(GoalMidFootPosition, dataSet.getPlannerInput().getGoalPosition());
            messager.submitMessage(GoalMidFootOrientation, new Quaternion(dataSet.getPlannerInput().getGoalYaw(), 0.0, 0.0));
         }

         messager.submitMessage(GlobalReset, true);
      };

      Consumer<HeightMapDataSetName> heightMapDataSetLoader = dataSetName ->
      {
         HeightMapMessage message = dataSetName.getMessage();
         messager.submitMessage(HeightMapData, message);

         RigidBodyTransform robotRootJoint = new RigidBodyTransform();
         robotRootJoint.getTranslation().set(dataSetName.getStart().getPosition());
         robotRootJoint.getRotation().setYawPitchRoll(dataSetName.getStart().getYaw(), 0.0, 0.0);
         robotRootJoint.getTranslation().sub(auxiliaryRobotData.getRootJointToMidFootOffset());
         robotVisualizer.submitNewConfiguration(robotRootJoint, auxiliaryRobotData.getDefaultJointAngleMap());

         if (robotIKVisualizer != null)
            robotIKVisualizer.submitNewConfiguration(robotRootJoint, auxiliaryRobotData.getDefaultJointAngleMap());

         messager.submitMessage(GoalMidFootPosition, dataSetName.getGoal().getPosition());
         messager.submitMessage(GoalMidFootOrientation, new Quaternion(dataSetName.getGoal().getYaw(), 0.0, 0.0));
         messager.submitMessage(GlobalReset, true);
      };

      messager.registerTopicListener(DataSetSelected, dataSetName -> Platform.runLater(() -> dataSetLoader.accept(dataSetName)));
      messager.registerTopicListener(HeightMapDataSetSelected, dataSetName -> Platform.runLater(() -> heightMapDataSetLoader.accept(dataSetName)));
   }

   public void setRobotLowLevelMessenger(RobotLowLevelMessenger robotLowLevelMessenger)
   {
      robotOperationTabController.setRobotLowLevelMessenger(robotLowLevelMessenger);
   }

   public void setREAStateRequestPublisher(IHMCRealtimeROS2Publisher<REAStateRequestMessage> reaStateRequestPublisher)
   {
      robotOperationTabController.setREAStateRequestPublisher(reaStateRequestPublisher);
   }

   public JavaFXMessager getMessager()
   {
      return messager;
   }

   public void show()
   {
      primaryStage.show();

      footstepPlannerLogVisualizerController.onPrimaryStageLoaded();
      bodyPathLogVisualizerController.onPrimaryStageLoaded();
      footstepPlannerParametersUIController.onPrimaryStageLoaded();
      visibilityGraphsParametersUIController.onPrimaryStageLoaded();
      swingPlannerParametersUIController.onPrimaryStageLoaded();
   }

   public void stop()
   {
      shutdownHooks.forEach(Runnable::run);

      planarRegionViewer.stop();
      ocTreeViewer.stop();
      startGoalPositionViewer.stop();
      goalOrientationViewer.stop();
      startGoalEditor.stop();
      orientationEditor.stop();
      pathViewer.stop();
      postProcessingViewer.stop();
      bodyPathMeshViewer.stop();
      visibilityGraphsRenderer.stop();
      heightMapVisualizer.stop();

      if (heightMapNavigationUpdater != null)
      {
         heightMapNavigationUpdater.stop();
      }

      if (robotVisualizer != null)
         robotVisualizer.stop();
   }

   public void addShutdownHook(Runnable shutdownHook)
   {
      shutdownHooks.add(shutdownHook);
   }

   public static FootstepPlannerUI createUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return createUI(primaryStage, messager, false, null);
   }

   public static FootstepPlannerUI createUI(Stage primaryStage,
                                            JavaFXMessager messager,
                                            boolean showTestDashboard,
                                            SideDependentList<List<Point2D>> defaultContactPoints)
         throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager, showTestDashboard, defaultContactPoints);
   }

   public static FootstepPlannerUI createUI(Stage primaryStage,
                                            JavaFXMessager messager,
                                            VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                            FootstepPlannerParametersBasics plannerParameters,
                                            SwingPlannerParametersBasics swingPlannerParameters,
                                            FullHumanoidRobotModelFactory fullHumanoidRobotModelFactory,
                                            FullHumanoidRobotModelFactory previewModelFactory,
                                            HumanoidJointNameMap jointMap,
                                            RobotContactPointParameters<RobotSide> contactPointParameters,
                                            WalkingControllerParameters walkingControllerParameters,
                                            UIAuxiliaryRobotData auxiliaryRobotData,
                                            CollisionBoxProvider collisionBoxProvider)
         throws Exception
   {
      SideDependentList<List<Point2D>> defaultContactPoints = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         defaultContactPoints.put(side, contactPointParameters.getControllerFootGroundContactPoints().get(side));
      }

      return new FootstepPlannerUI(primaryStage,
                                   messager,
                                   visibilityGraphsParameters,
                                   plannerParameters,
                                   swingPlannerParameters,
                                   fullHumanoidRobotModelFactory,
                                   previewModelFactory,
                                   jointMap,
                                   walkingControllerParameters,
                                   auxiliaryRobotData,
                                   false,
                                   defaultContactPoints,
                                   collisionBoxProvider);
   }
}
