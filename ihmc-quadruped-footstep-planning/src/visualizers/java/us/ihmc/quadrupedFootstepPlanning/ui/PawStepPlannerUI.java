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
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.DefaultPawStepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.tools.PawStepPlannerDataExporter;
import us.ihmc.quadrupedFootstepPlanning.ui.components.NodeCheckerEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.*;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.*;
import us.ihmc.robotModels.FullQuadrupedRobotModelFactory;

import static us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI.*;

/**
 * This class is the visualization element of the footstep planner. It also contains a graphical interface for
 * setting planner parameters to be used by the footstep planner itself.
 */
public class PawStepPlannerUI
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
   private final PawStepPlannerDataExporter dataExporter;
   private final BodyPawPathMeshViewer bodyPathMeshViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
//   private final NodeOccupancyMapRenderer graphRenderer;
   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final JavaFXQuadrupedVisualizer walkingPreviewVisualizer;
   private final PawStepPlannerProcessViewer pawPlannerProcessViewer;

   @FXML
   private PawStepPlannerMenuUIController pawStepPlannerMenuUIController;
   @FXML
   private PawNodeCheckingUIController pawNodeCheckingUIController;
   @FXML
   private PawStepPlannerParametersUIController pawPlannerParametersUIController;
   @FXML
   private PlannerReachParametersUIController plannerReachParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private PawStepPlannerDataExporterAnchorPaneController dataExporterAnchorPaneController;
   @FXML
   private MainTabController mainTabController;
   @FXML
   private PawStepPlannerVisualizationController pawStepPlannerVizController;
   @FXML
   private VisualizationController visibilityGraphsVizController;

   public PawStepPlannerUI(Stage primaryStage, PawStepPlannerParametersBasics plannerParameters, VisibilityGraphsParametersBasics visibilityGraphsParameters) throws Exception
   {
      this(primaryStage, new SharedMemoryJavaFXMessager(PawStepPlannerMessagerAPI.API), plannerParameters, visibilityGraphsParameters, null);
      messager.startMessager();
   }

   public PawStepPlannerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      this(primaryStage, messager, new DefaultPawStepPlannerParameters(), new DefaultVisibilityGraphParameters(), null);
   }

   public PawStepPlannerUI(Stage primaryStage, JavaFXMessager messager, PawStepPlannerParametersBasics plannerParameters,
                           VisibilityGraphsParametersBasics visibilityGraphsParameters, FullQuadrupedRobotModelFactory fullQuadrupedRobotModelFactory) throws Exception
   {
      this(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullQuadrupedRobotModelFactory, null);
   }

   public PawStepPlannerUI(Stage primaryStage, JavaFXMessager messager, PawStepPlannerParametersBasics plannerParameters,
                           VisibilityGraphsParametersBasics visibilityGraphsParameters, FullQuadrupedRobotModelFactory fullQuadrupedRobotModelFactory,
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
      pawStepPlannerMenuUIController.attachMessager(messager);
      pawPlannerParametersUIController.attachMessager(messager);
      plannerReachParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      pawNodeCheckingUIController.attachMessager(messager);
      dataExporterAnchorPaneController.attachMessager(messager);
      pawStepPlannerVizController.attachMessager(messager);
      visibilityGraphsVizController.attachMessager(messager);

      setMainTabTopics();
      pawPlannerParametersUIController.setPlannerParametersTopic(PawStepPlannerMessagerAPI.PlannerParametersTopic);
      plannerReachParametersUIController.setPlannerParametersTopic(PawStepPlannerMessagerAPI.PlannerParametersTopic);
      visibilityGraphsParametersUIController.setVisibilityGraphsParametersTopic(PawStepPlannerMessagerAPI.VisibilityGraphsParametersTopic);

      pawStepPlannerMenuUIController.setMainWindow(primaryStage);

      mainTabController.bindControls();
      pawPlannerParametersUIController.bindControls();
      plannerReachParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      pawNodeCheckingUIController.bindControls();
      visibilityGraphsVizController.bindControls();
      pawStepPlannerVizController.bindControls();

      pawPlannerParametersUIController.loadFromFile();
      plannerReachParametersUIController.loadFromFile();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPawPositionViewer(messager, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                                    StartPositionTopic, StartOrientationTopic, LowLevelGoalPositionTopic, GoalPositionTopic,
                                                                    GoalOrientationTopic, StartFeetPositionTopic, XGaitSettingsTopic, PlanarRegionDataTopic);
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
      this.dataExporter = new PawStepPlannerDataExporter(messager);
      this.bodyPathMeshViewer = new BodyPawPathMeshViewer(messager, ShowBodyPathTopic, ComputePathTopic, BodyPathDataTopic);
      this.visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
//      this.graphRenderer = new NodeOccupancyMapRenderer(messager);
      this.pawPlannerProcessViewer = includeProcessView ? new PawStepPlannerProcessViewer(messager) : null;

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
      mainTabController.setPlannerTypeTopic(PawStepPlannerMessagerAPI.PlannerTypeTopic);
      mainTabController.setPlannerRequestIdTopic(PawStepPlannerMessagerAPI.PlannerRequestIdTopic);
      mainTabController.setReceivedPlanIdTopic(PawStepPlannerMessagerAPI.ReceivedPlanIdTopic);
      mainTabController.setFootstepPlanTopic(PawStepPlannerMessagerAPI.ShowFootstepPlanTopic, PawStepPlannerMessagerAPI.FootstepPlanTopic);
      mainTabController.setPlanarRegionDataTopic(PawStepPlannerMessagerAPI.PlanarRegionDataTopic);
      mainTabController.setPlannerTimeTakenTopic(PawStepPlannerMessagerAPI.PlannerTimeTakenTopic);
      mainTabController.setPlannerTimeoutTopic(PawStepPlannerMessagerAPI.PlannerTimeoutTopic);
      mainTabController.setComputePathTopic(PawStepPlannerMessagerAPI.ComputePathTopic);
      mainTabController.setAbortPlanningTopic(PawStepPlannerMessagerAPI.AbortPlanningTopic);
      mainTabController.setAcceptNewPlanarRegionsTopic(PawStepPlannerMessagerAPI.AcceptNewPlanarRegionsTopic);
      mainTabController.setPlanningResultTopic(PawStepPlannerMessagerAPI.PlanningResultTopic);
      mainTabController.setPlannerStatusTopic(PawStepPlannerMessagerAPI.PlannerStatusTopic);
      mainTabController.setPlannerHorizonLengthTopic(PawStepPlannerMessagerAPI.PlannerHorizonLengthTopic);
      mainTabController.setStartGoalTopics(PawStepPlannerMessagerAPI.EditModeEnabledTopic, PawStepPlannerMessagerAPI.StartPositionEditModeEnabledTopic,
                                           PawStepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, PawStepPlannerMessagerAPI.InitialSupportQuadrantTopic,
                                           PawStepPlannerMessagerAPI.StartPositionTopic, PawStepPlannerMessagerAPI.StartOrientationTopic,
                                           PawStepPlannerMessagerAPI.GoalPositionTopic, PawStepPlannerMessagerAPI.GoalOrientationTopic,
                                           PawStepPlannerMessagerAPI.StartTargetTypeTopic, PawStepPlannerMessagerAPI.StartFeetPositionTopic);
      mainTabController.setAssumeFlatGroundTopic(PawStepPlannerMessagerAPI.AssumeFlatGroundTopic);
      mainTabController.setGlobalResetTopic(PawStepPlannerMessagerAPI.GlobalResetTopic);
      mainTabController.setPlannerPlaybackFractionTopic(PawStepPlannerMessagerAPI.PlannerPlaybackFractionTopic);
      mainTabController.setXGaitSettingsTopic(PawStepPlannerMessagerAPI.XGaitSettingsTopic);
      mainTabController.setShowFootstepPreviewTopic(PawStepPlannerMessagerAPI.ShowFootstepPreviewTopic);
      mainTabController.setStepListMessageTopic(PawStepPlannerMessagerAPI.FootstepDataListTopic);
   }

   public static PawStepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return new PawStepPlannerUI(primaryStage, messager);
   }

   public static PawStepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager, PawStepPlannerParametersBasics plannerParameters,
                                                   VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                                   FullQuadrupedRobotModelFactory fullQuadrupedRobotModelFactory,
                                                   FullQuadrupedRobotModelFactory previewModelFactory)
         throws Exception
   {
      return new PawStepPlannerUI(primaryStage, messager, plannerParameters, visibilityGraphsParameters, fullQuadrupedRobotModelFactory, previewModelFactory);
   }
}
