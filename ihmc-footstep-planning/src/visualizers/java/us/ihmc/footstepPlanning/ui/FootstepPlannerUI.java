package us.ihmc.footstepPlanning.ui;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.DataSetSelected;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GlobalReset;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalMidFootOrientation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalMidFootPosition;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalPositionEditModeEnabled;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.HeightMapData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.HeightMapDataSetSelected;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.LowLevelGoalPosition;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.OcTreeData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanarRegionData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RobotConfigurationData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.SelectedRegion;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowCoordinateSystem;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowGoal;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowHeightMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowOcTree;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowPlanarRegions;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowRobot;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowStart;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.REAStateRequestMessage;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.footstepPlanning.ui.components.FootPoseFromMidFootUpdater;
import us.ihmc.footstepPlanning.ui.components.FootstepCompletionListener;
import us.ihmc.footstepPlanning.ui.components.GoalOrientationEditor;
import us.ihmc.footstepPlanning.ui.components.HeightMapNavigationUpdater;
import us.ihmc.footstepPlanning.ui.components.ManualFootstepAdjustmentListener;
import us.ihmc.footstepPlanning.ui.components.UIFootstepPlanManager;
import us.ihmc.footstepPlanning.ui.controllers.AStarBodyPathParametersUIController;
import us.ihmc.footstepPlanning.ui.controllers.BodyPathLogVisualizerController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerLogVisualizerController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerMenuUIController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerParametersUIController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerStatusBarController;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerTestDashboardController;
import us.ihmc.footstepPlanning.ui.controllers.MainTabController;
import us.ihmc.footstepPlanning.ui.controllers.RobotOperationTabController;
import us.ihmc.footstepPlanning.ui.controllers.SwingPlannerParametersUIController;
import us.ihmc.footstepPlanning.ui.controllers.VisibilityGraphsParametersUIController;
import us.ihmc.footstepPlanning.ui.controllers.VisualizationController;
import us.ihmc.footstepPlanning.ui.viewers.BodyPathLogRenderer;
import us.ihmc.footstepPlanning.ui.viewers.BodyPathMeshViewer;
import us.ihmc.footstepPlanning.ui.viewers.FootstepPathMeshViewer;
import us.ihmc.footstepPlanning.ui.viewers.FootstepPlannerLogRenderer;
import us.ihmc.footstepPlanning.ui.viewers.GoalOrientationViewer;
import us.ihmc.footstepPlanning.ui.viewers.HeightMapVisualizer;
import us.ihmc.footstepPlanning.ui.viewers.RobotIKVisualizer;
import us.ihmc.footstepPlanning.ui.viewers.StartGoalPositionViewer;
import us.ihmc.footstepPlanning.ui.viewers.SwingPlanMeshViewer;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXCoordinateSystem;
import us.ihmc.javafx.JavaFXRobotVisualizer;
import us.ihmc.messager.javafx.JavaFXMessager;
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
import us.ihmc.wholeBodyController.RobotContactPointParameters;

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
   private AStarBodyPathParametersUIController aStarBodyPathParametersUIController;
   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;
   @FXML
   private SwingPlannerParametersUIController swingPlannerParametersUIController;
   @FXML
   private FootstepPlannerLogVisualizerController footstepPlannerLogVisualizerController;
   @FXML
   private BodyPathLogVisualizerController bodyPathLogVisualizerController;
   @FXML
   private VisualizationController visibilityGraphsUIController;

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, boolean showTestDashboard, SideDependentList<List<Point2D>> defaultContactPoints)
         throws Exception
   {
      this(primaryStage,
           messager,
           new AStarBodyPathPlannerParameters(),
           new DefaultFootstepPlannerParameters(),
           new DefaultSwingPlannerParameters(),
           null,
           null,
           showTestDashboard,
           defaultContactPoints);
   }

   public FootstepPlannerUI(Stage primaryStage,
                            JavaFXMessager messager,
                            AStarBodyPathPlannerParametersBasics aStarBodyPathParameters,
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
           aStarBodyPathParameters,
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
                            AStarBodyPathPlannerParametersBasics aStarBodyPathParameters,
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

      aStarBodyPathParametersUIController.setAStarBodyPathParameters(aStarBodyPathParameters);
      footstepPlannerParametersUIController.setPlannerParameters(plannerParameters);
      swingPlannerParametersUIController.setParameters(swingPlannerParameters);

      mainTabController.attachMessager(messager);
      footstepPlannerStatusBarController.attachMessager(messager);
      footstepPlannerTestDashboardController.attachMessager(messager);
      footstepPlannerMenuUIController.attachMessager(messager);
      aStarBodyPathParametersUIController.attachMessager(messager);
      footstepPlannerParametersUIController.attachMessager(messager);
      bodyPathLogVisualizerController.attachMessager(messager);
      swingPlannerParametersUIController.attachMessager(messager);
      footstepPlannerLogVisualizerController.attachMessager(messager);
      visibilityGraphsUIController.attachMessager(messager);

      if (ENABLE_HEIGHT_MAP_VIZ)
      {
         messager.addTopicListener(HeightMapData, heightMapVisualizer::update);
      }

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      mainTabController.bindControls();
      footstepPlannerStatusBarController.bindControls();
      bodyPathLogVisualizerController.bindControls();
      footstepPlannerTestDashboardController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      aStarBodyPathParametersUIController.bindControls();
      footstepPlannerLogVisualizerController.bindControls();
      swingPlannerParametersUIController.bindControls();
      visibilityGraphsUIController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addDefaultLighting();

      JavaFXCoordinateSystem worldCoordinateSystem = new JavaFXCoordinateSystem(0.3);
      worldCoordinateSystem.setMouseTransparent(true);
      view3dFactory.addNodeToView(worldCoordinateSystem);
      messager.addTopicListener(ShowCoordinateSystem, worldCoordinateSystem::setVisible);

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
      this.footstepPlannerLogRenderer = new FootstepPlannerLogRenderer(defaultContactPoints, messager);
      this.bodyPathLogRenderer = new BodyPathLogRenderer(messager);
      new UIFootstepPlanManager(messager);
      this.manualFootstepAdjustmentListener = new ManualFootstepAdjustmentListener(messager, view3dFactory.getSubScene());

      startGoalPositionViewer.setShowStartGoalTopics(ShowStart, ShowGoal, ShowGoal);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(ocTreeViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(goalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathViewer.getRoot());
      view3dFactory.addNodeToView(postProcessingViewer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
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
         messager.addTopicListener(RobotConfigurationData, robotVisualizer::submitNewConfiguration);

         mainTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
         messager.addTopicListener(ShowRobot, show -> robotVisualizer.getRootNode().setVisible(show));
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

      if (defaultContactPoints == null)
      {
         SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
         defaultContactPoints = new SideDependentList<>();
         for (RobotSide side : RobotSide.values)
            defaultContactPoints.put(side, footPolygons.get(side).getPolygonVerticesView().stream().map(Point2D::new).collect(Collectors.toList()));
      }

      mainTabController.setContactPointParameters(defaultContactPoints);
      pathViewer.setDefaultContactPoints(defaultContactPoints);
      goalOrientationViewer.setDefaultContactPoints(defaultContactPoints);
      footstepPlannerLogVisualizerController.setContactPointParameters(defaultContactPoints);


      messager.addTopicListener(ShowOcTree, ocTreeViewer::setEnabled);
      messager.addTopicListener(OcTreeData, ocTreeViewer::submitOcTreeData);

      planarRegionViewer.start();
      ocTreeViewer.start();
      startGoalPositionViewer.start();
      goalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathViewer.start();
      postProcessingViewer.start();
      bodyPathMeshViewer.start();
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

      messager.addTopicListener(HeightMapData, data -> planarRegionViewer.clear());
      messager.addTopicListener(PlanarRegionData, data -> heightMapVisualizer.clear());
      messager.addTopicListener(ShowHeightMap, show -> heightMapVisualizer.getRoot().setVisible(show));
      setupDataSetLoadBingings(auxiliaryRobotData);

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
         if (dataSet.hasPlannerInput() && auxiliaryRobotData != null)
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

         if (auxiliaryRobotData != null)
         {
            RigidBodyTransform robotRootJoint = new RigidBodyTransform();
            robotRootJoint.getTranslation().set(dataSetName.getStart().getPosition());
            robotRootJoint.getRotation().setYawPitchRoll(dataSetName.getStart().getYaw(), 0.0, 0.0);
            robotRootJoint.getTranslation().sub(auxiliaryRobotData.getRootJointToMidFootOffset());
            robotVisualizer.submitNewConfiguration(robotRootJoint, auxiliaryRobotData.getDefaultJointAngleMap());

            if (robotIKVisualizer != null)
               robotIKVisualizer.submitNewConfiguration(robotRootJoint, auxiliaryRobotData.getDefaultJointAngleMap());
         }

         messager.submitMessage(GoalMidFootPosition, dataSetName.getGoal().getPosition());
         messager.submitMessage(GoalMidFootOrientation, new Quaternion(dataSetName.getGoal().getYaw(), 0.0, 0.0));
         messager.submitMessage(GlobalReset, true);
      };

      messager.addTopicListener(DataSetSelected, dataSetName -> Platform.runLater(() -> dataSetLoader.accept(dataSetName)));
      messager.addTopicListener(HeightMapDataSetSelected, dataSetName -> Platform.runLater(() -> heightMapDataSetLoader.accept(dataSetName)));
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
      aStarBodyPathParametersUIController.onPrimaryStageLoaded();
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
                                            AStarBodyPathPlannerParametersBasics aStarBodyPathParameters,
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
                                   aStarBodyPathParameters,
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
