package us.ihmc.quadrupedFootstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXQuadrupedVisualizer;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools.PawPlannerDataExporter;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeCheckerEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.*;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.*;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;

import static us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI.*;

/**
 * This class is the visualization element of the footstep planner. It also contains a graphical interface for
 * setting planner parameters to be used by the footstep planner itself.
 */
public class PawPlannerUI
{
   private static final boolean VERBOSE = true;
   private static final boolean includeProcessView = true;

   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final NodeCheckerEditor nodeCheckerEditor;
   private final StartGoalPawPositionViewer startGoalPositionViewer;
   private final StartGoalPawOrientationViewer startGoalOrientationViewer;
   private final PawPathMeshViewer pathViewer;
   private final StartGoalOrientationEditor orientationEditor;
   private final PawNodeCheckerRenderer nodeCheckerRenderer;
   private final PawPlannerDataExporter dataExporter;
   private final BodyPawPathMeshViewer bodyPathMeshViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
//   private final NodeOccupancyMapRenderer graphRenderer;
   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final JavaFXQuadrupedVisualizer walkingPreviewVisualizer;
   private final PawPlannerProcessViewer pawPlannerProcessViewer;

   @FXML
   private PawPlannerMenuUIController pawPlannerMenuUIController;
   @FXML
   private PawNodeCheckingUIController pawNodeCheckingUIController;
   @FXML
   private PawPlannerParametersUIController pawPlannerParametersUIController;
   @FXML
   private PlannerReachParametersUIController plannerReachParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private PawPlannerDataExporterAnchorPaneController dataExporterAnchorPaneController;
   @FXML
   private MainTabController mainTabController;
   @FXML
   private PawPlannerVisualizationController footstepPlannerVizController;
   @FXML
   private VisualizationController visibilityGraphsVizController;

   public PawPlannerUI(Stage primaryStage, PawPlannerParametersBasics plannerParameters, VisibilityGraphsParameters visibilityGraphsParameters) throws Exception
   {
      this(primaryStage, new SharedMemoryJavaFXMessager(PawPlannerMessagerAPI.API), plannerParameters, visibilityGraphsParameters, null);
      messager.startMessager();
   }

   public PawPlannerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      this(primaryStage, messager, new DefaultPawPlannerParameters(), new DefaultVisibilityGraphParameters(), null);
   }

   public PawPlannerUI(Stage primaryStage, JavaFXMessager messager, PawPlannerParametersBasics plannerParameters,
                       VisibilityGraphsParameters visibilityGraphsParameters, FullQuadrupedRobotModelFactory fullQuadrupedRobotModelFactory) throws Exception
   {
      this(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullQuadrupedRobotModelFactory, null);
   }

   public PawPlannerUI(Stage primaryStage, JavaFXMessager messager, PawPlannerParametersBasics plannerParameters,
                       VisibilityGraphsParameters visibilityGraphsParameters, FullQuadrupedRobotModelFactory fullQuadrupedRobotModelFactory,
                       FullQuadrupedRobotModelFactory previewModelFactory) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      pawPlannerParametersUIController.setPlannerParameters(plannerParameters);
      plannerReachParametersUIController.setPlannerParameters(plannerParameters);
      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);

      mainTabController.attachMessager(messager);
      pawPlannerMenuUIController.attachMessager(messager);
      pawPlannerParametersUIController.attachMessager(messager);
      plannerReachParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      pawNodeCheckingUIController.attachMessager(messager);
      dataExporterAnchorPaneController.attachMessager(messager);
      footstepPlannerVizController.attachMessager(messager);
      visibilityGraphsVizController.attachMessager(messager);

      setMainTabTopics();
      pawPlannerParametersUIController.setPlannerParametersTopic(PawPlannerMessagerAPI.PlannerParametersTopic);
      plannerReachParametersUIController.setPlannerParametersTopic(PawPlannerMessagerAPI.PlannerParametersTopic);
      visibilityGraphsParametersUIController.setVisibilityGraphsParametersTopic(PawPlannerMessagerAPI.VisibilityGraphsParametersTopic);

      pawPlannerMenuUIController.setMainWindow(primaryStage);

      mainTabController.bindControls();
      pawPlannerParametersUIController.bindControls();
      plannerReachParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      pawNodeCheckingUIController.bindControls();
      visibilityGraphsVizController.bindControls();
      footstepPlannerVizController.bindControls();

      pawPlannerParametersUIController.loadFromFile();
      plannerReachParametersUIController.loadFromFile();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPawPositionViewer(messager, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                                    StartPositionTopic, StartOrientationTopic, LowLevelGoalPositionTopic, GoalPositionTopic,
                                                                    GoalOrientationTopic, XGaitSettingsTopic, PlanarRegionDataTopic);
      this.startGoalOrientationViewer = new StartGoalPawOrientationViewer(messager, StartOrientationEditModeEnabledTopic, GoalOrientationEditModeEnabledTopic,
                                                                          StartPositionTopic, StartOrientationTopic, LowLevelGoalPositionTopic,
                                                                          LowLevelGoalOrientationTopic, GoalPositionTopic, GoalOrientationTopic);
      this.startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                         StartPositionTopic, GoalPositionTopic, PlanarRegionDataTopic, SelectedRegionTopic,
                                                         StartOrientationEditModeEnabledTopic, GoalOrientationEditModeEnabledTopic);
      this.orientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene(), EditModeEnabledTopic, StartOrientationEditModeEnabledTopic,
                                                              GoalOrientationEditModeEnabledTopic, StartPositionTopic, StartOrientationTopic,
                                                              GoalPositionTopic, GoalOrientationTopic, SelectedRegionTopic);
      this.nodeCheckerEditor = new NodeCheckerEditor(messager, subScene);
      this.pathViewer = new PawPathMeshViewer(messager, FootstepPlanTopic, ComputePathTopic, ShowFootstepPlanTopic, ShowFootstepPreviewTopic);
      this.nodeCheckerRenderer = new PawNodeCheckerRenderer(messager);
      this.dataExporter = new PawPlannerDataExporter(messager);
      this.bodyPathMeshViewer = new BodyPawPathMeshViewer(messager, ShowBodyPathTopic, ComputePathTopic, BodyPathDataTopic);
      this.visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
//      this.graphRenderer = new NodeOccupancyMapRenderer(messager);
      this.pawPlannerProcessViewer = includeProcessView ? new PawPlannerProcessViewer(messager) : null;

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathViewer.getRoot());
      view3dFactory.addNodeToView(nodeCheckerRenderer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(visibilityGraphsRenderer.getRoot());
//      view3dFactory.addNodeToView(graphRenderer.getRoot());
      if (includeProcessView)
         view3dFactory.addNodeToView(pawPlannerProcessViewer.getRoot());

      if(fullQuadrupedRobotModelFactory == null)
      {
         robotVisualizer = null;
      }
      else
      {
         robotVisualizer = new JavaFXQuadrupedVisualizer(fullQuadrupedRobotModelFactory);
         messager.registerTopicListener(RobotConfigurationDataTopic, robotVisualizer::submitNewConfiguration);
         mainTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());
         view3dFactory.addNodeToView(robotVisualizer.getRootNode());
         robotVisualizer.start();
      }

      if(previewModelFactory == null)
      {
         walkingPreviewVisualizer = null;
      }
      else
      {
         walkingPreviewVisualizer = new JavaFXQuadrupedVisualizer(previewModelFactory);
         view3dFactory.addNodeToView(walkingPreviewVisualizer.getRootNode());
         mainTabController.setPreviewModel(walkingPreviewVisualizer.getFullRobotModel());
         walkingPreviewVisualizer.getFullRobotModel().getRootJoint().setJointPosition(new Vector3D(Double.NaN, Double.NaN, Double.NaN));
         walkingPreviewVisualizer.start();
      }


      mainTabController.setPreviewFootstepPositions(pathViewer.getPreviewFootstepPositions());


      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathViewer.start();
      nodeCheckerRenderer.start();
      nodeCheckerEditor.start();
      bodyPathMeshViewer.start();
      visibilityGraphsRenderer.start();
//      graphRenderer.start();
      if (pawPlannerProcessViewer != null)
         pawPlannerProcessViewer.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }



   public JavaFXMessager getMessager()
   {
      return messager;
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
      pathViewer.stop();
      nodeCheckerRenderer.stop();
      dataExporter.stop();
      bodyPathMeshViewer.stop();
      visibilityGraphsRenderer.stop();
      if (pawPlannerProcessViewer != null)
         pawPlannerProcessViewer.stop();

      if(robotVisualizer != null)
         robotVisualizer.stop();
   }

   private void setMainTabTopics()
   {
      mainTabController.setPlannerTypeTopic(PawPlannerMessagerAPI.PlannerTypeTopic);
      mainTabController.setPlannerRequestIdTopic(PawPlannerMessagerAPI.PlannerRequestIdTopic);
      mainTabController.setReceivedPlanIdTopic(PawPlannerMessagerAPI.ReceivedPlanIdTopic);
      mainTabController.setFootstepPlanTopic(PawPlannerMessagerAPI.ShowFootstepPlanTopic, PawPlannerMessagerAPI.FootstepPlanTopic);
      mainTabController.setPlanarRegionDataTopic(PawPlannerMessagerAPI.PlanarRegionDataTopic);
      mainTabController.setPlannerTimeTakenTopic(PawPlannerMessagerAPI.PlannerTimeTakenTopic);
      mainTabController.setPlannerTimeoutTopic(PawPlannerMessagerAPI.PlannerTimeoutTopic);
      mainTabController.setComputePathTopic(PawPlannerMessagerAPI.ComputePathTopic);
      mainTabController.setAbortPlanningTopic(PawPlannerMessagerAPI.AbortPlanningTopic);
      mainTabController.setAcceptNewPlanarRegionsTopic(PawPlannerMessagerAPI.AcceptNewPlanarRegionsTopic);
      mainTabController.setPlanningResultTopic(PawPlannerMessagerAPI.PlanningResultTopic);
      mainTabController.setPlannerStatusTopic(PawPlannerMessagerAPI.PlannerStatusTopic);
      mainTabController.setPlannerHorizonLengthTopic(PawPlannerMessagerAPI.PlannerHorizonLengthTopic);
      mainTabController.setStartGoalTopics(PawPlannerMessagerAPI.EditModeEnabledTopic, PawPlannerMessagerAPI.StartPositionEditModeEnabledTopic,
                                           PawPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, PawPlannerMessagerAPI.InitialSupportQuadrantTopic,
                                           PawPlannerMessagerAPI.StartPositionTopic, PawPlannerMessagerAPI.StartOrientationTopic,
                                           PawPlannerMessagerAPI.GoalPositionTopic, PawPlannerMessagerAPI.GoalOrientationTopic,
                                           PawPlannerMessagerAPI.StartTargetTypeTopic, PawPlannerMessagerAPI.StartFeetPositionTopic);
      mainTabController.setAssumeFlatGroundTopic(PawPlannerMessagerAPI.AssumeFlatGroundTopic);
      mainTabController.setGlobalResetTopic(PawPlannerMessagerAPI.GlobalResetTopic);
      mainTabController.setPlannerPlaybackFractionTopic(PawPlannerMessagerAPI.PlannerPlaybackFractionTopic);
      mainTabController.setXGaitSettingsTopic(PawPlannerMessagerAPI.XGaitSettingsTopic);
      mainTabController.setShowFootstepPreviewTopic(PawPlannerMessagerAPI.ShowFootstepPreviewTopic);
      mainTabController.setStepListMessageTopic(PawPlannerMessagerAPI.FootstepDataListTopic);
   }

   public static PawPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return new PawPlannerUI(primaryStage, messager);
   }

   public static PawPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager, PawPlannerParametersBasics plannerParameters,
                                               VisibilityGraphsParameters visibilityGraphsParameters,
                                               FullQuadrupedRobotModelFactory fullQuadrupedRobotModelFactory,
                                               FullQuadrupedRobotModelFactory previewModelFactory)
         throws Exception
   {
      return new PawPlannerUI(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullQuadrupedRobotModelFactory, previewModelFactory);
   }
}
