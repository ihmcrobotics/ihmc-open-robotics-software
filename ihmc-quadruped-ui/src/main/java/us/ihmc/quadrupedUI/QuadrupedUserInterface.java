package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXQuadrupedVisualizer;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.FootstepPlannerMenuUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.FootstepPlannerParametersUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.MainTabController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.VisibilityGraphsParametersUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.BodyPathMeshViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.FootstepPathMeshViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.StartGoalOrientationViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.StartGoalPositionViewer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedUI.uiControllers.RobotControlTabController;
import us.ihmc.quadrupedUI.uiControllers.ManualStepTabController;
import us.ihmc.quadrupedUI.uiControllers.XGaitSettingsController;

public class QuadrupedUserInterface
{
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final StartGoalOrientationViewer startGoalOrientationViewer;
   private final StartGoalPositionEditor startGoalPositionEditor;
   private final StartGoalOrientationEditor startGoalOrientationEditor;
   private final FootstepPathMeshViewer pawPathViewer;
   private final BodyPathMeshViewer bodyPathMeshViewer;


   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final AnimationTimer cameraTracking;

   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;
   @FXML
   private MainTabController plannerTabController;
   @FXML
   private RobotControlTabController robotControlTabController;
   @FXML
   private XGaitSettingsController xGaitSettingsController;
   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private ManualStepTabController manualStepTabController;

   public QuadrupedUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                 FootstepPlannerParameters footstepPlannerParameters, VisibilityGraphsParameters visibilityGraphsParameters,
                                 QuadrupedXGaitSettingsReadOnly xGaitSettings)
         throws Exception
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      footstepPlannerParametersUIController.setPlannerParameters(footstepPlannerParameters);
      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);

      plannerTabController.attachMessager(messager);
      robotControlTabController.attachMessager(messager);
      xGaitSettingsController.attachMessager(messager, xGaitSettings);
      footstepPlannerParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      manualStepTabController.attachMessager(messager, xGaitSettings);

      manualStepTabController.setFullRobotModelFactory(modelFactory);

      setPlannerTabTopics();
      footstepPlannerParametersUIController.setPlannerParametersTopic(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic);
      visibilityGraphsParametersUIController.setVisibilityGraphsParametersTopic(QuadrupedUIMessagerAPI.VisibilityGraphsParametersTopic);

      plannerTabController.bindControls();
      robotControlTabController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      xGaitSettingsController.bindControls();
      manualStepTabController.bindControls();

      footstepPlannerParametersUIController.loadFromFile();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, QuadrupedUIMessagerAPI.PlanarRegionDataTopic, QuadrupedUIMessagerAPI.ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPositionViewer(messager, QuadrupedUIMessagerAPI.StartPositionEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.GoalPositionEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                                                 QuadrupedUIMessagerAPI.LowLevelGoalPositionTopic, QuadrupedUIMessagerAPI.GoalPositionTopic,
                                                                 QuadrupedUIMessagerAPI.GoalOrientationTopic, QuadrupedUIMessagerAPI.XGaitSettingsTopic,
                                                                 QuadrupedUIMessagerAPI.PlanarRegionDataTopic);
      this.startGoalOrientationViewer = new StartGoalOrientationViewer(messager, QuadrupedUIMessagerAPI.StartOrientationEditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.GoalOrientationEditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                                                       QuadrupedUIMessagerAPI.LowLevelGoalPositionTopic,
                                                                       QuadrupedUIMessagerAPI.LowLevelGoalOrientationTopic,
                                                                       QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic);
      this.startGoalPositionEditor = new StartGoalPositionEditor(messager, subScene, QuadrupedUIMessagerAPI.StartPositionEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.GoalPositionEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.GoalPositionTopic,
                                                                 QuadrupedUIMessagerAPI.PlanarRegionDataTopic, QuadrupedUIMessagerAPI.SelectedRegionTopic,
                                                                 QuadrupedUIMessagerAPI.StartOrientationEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.GoalOrientationEditModeEnabledTopic);
      this.startGoalOrientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene(), QuadrupedUIMessagerAPI.EditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.StartOrientationEditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.GoalOrientationEditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                                                       QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic,
                                                                       QuadrupedUIMessagerAPI.SelectedRegionTopic);
      this.pawPathViewer = new FootstepPathMeshViewer(messager, QuadrupedUIMessagerAPI.FootstepPlanTopic, QuadrupedUIMessagerAPI.ComputePathTopic,
                                                      QuadrupedUIMessagerAPI.ShowFootstepPlanTopic, QuadrupedUIMessagerAPI.ShowFootstepPreviewTopic);
      this.bodyPathMeshViewer = new BodyPathMeshViewer(messager, QuadrupedUIMessagerAPI.ShowBodyPathTopic, QuadrupedUIMessagerAPI.ComputePathTopic,
                                                       QuadrupedUIMessagerAPI.BodyPathDataTopic);


      robotVisualizer = new JavaFXQuadrupedVisualizer(messager, modelFactory, QuadrupedUIMessagerAPI.RobotModelTopic);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, this::submitNewConfiguration);

      plannerTabController.setFullRobotModel(robotVisualizer.getFullRobotModel());

      // just to get all the values out there.
      messager.submitMessage(QuadrupedUIMessagerAPI.XGaitSettingsTopic, xGaitSettings);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());
      view3dFactory.addNodeToView(pawPathViewer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());

      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
      Translate rootJointOffset = new Translate();
      cameraController.prependTransform(rootJointOffset);

      cameraTracking = new AnimationTimer()
      {
         @Override
         public void handle(long now)
         {
            FramePoint3D rootJointPosition = new FramePoint3D(robotVisualizer.getFullRobotModel().getRootJoint().getFrameAfterJoint());
            rootJointPosition.changeFrame(ReferenceFrame.getWorldFrame());
            rootJointOffset.setX(rootJointPosition.getX());
            rootJointOffset.setY(rootJointPosition.getY());
            rootJointOffset.setZ(rootJointPosition.getZ());
         }
      };


      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalPositionEditor.start();
      startGoalOrientationEditor.start();
      pawPathViewer.start();
      bodyPathMeshViewer.start();

      robotVisualizer.start();
      cameraTracking.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void submitNewConfiguration(RobotConfigurationData configuration)
   {
      robotVisualizer.submitNewConfiguration(configuration);
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
      startGoalPositionEditor.stop();
      startGoalOrientationEditor.stop();
      pawPathViewer.stop();
      bodyPathMeshViewer.stop();
      cameraTracking.stop();

      try
      {
         robotVisualizer.stop();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private void setPlannerTabTopics()
   {
      plannerTabController.setPlannerTypeTopic(QuadrupedUIMessagerAPI.PlannerTypeTopic);
      plannerTabController.setPlannerRequestIdTopic(QuadrupedUIMessagerAPI.PlannerRequestIdTopic);
      plannerTabController.setReceivedPlanIdTopic(QuadrupedUIMessagerAPI.ReceivedPlanIdTopic);
      plannerTabController.setFootstepPlanTopic(QuadrupedUIMessagerAPI.ShowFootstepPlanTopic, QuadrupedUIMessagerAPI.FootstepPlanTopic);
      plannerTabController.setPlanarRegionDataTopic(QuadrupedUIMessagerAPI.PlanarRegionDataTopic);
      plannerTabController.setPlannerTimeTakenTopic(QuadrupedUIMessagerAPI.PlannerTimeTakenTopic);
      plannerTabController.setPlannerTimeoutTopic(QuadrupedUIMessagerAPI.PlannerTimeoutTopic);
      plannerTabController.setComputePathTopic(QuadrupedUIMessagerAPI.ComputePathTopic);
      plannerTabController.setAbortPlanningTopic(QuadrupedUIMessagerAPI.AbortPlanningTopic);
      plannerTabController.setAcceptNewPlanarRegionsTopic(QuadrupedUIMessagerAPI.AcceptNewPlanarRegionsTopic);
      plannerTabController.setPlanningResultTopic(QuadrupedUIMessagerAPI.PlanningResultTopic);
      plannerTabController.setPlannerStatusTopic(QuadrupedUIMessagerAPI.PlannerStatusTopic);
      plannerTabController.setPlannerHorizonLengthTopic(QuadrupedUIMessagerAPI.PlannerHorizonLengthTopic);
      plannerTabController.setStartGoalTopics(QuadrupedUIMessagerAPI.EditModeEnabledTopic, QuadrupedUIMessagerAPI.StartPositionEditModeEnabledTopic,
                                              QuadrupedUIMessagerAPI.GoalPositionEditModeEnabledTopic, QuadrupedUIMessagerAPI.InitialSupportQuadrantTopic,
                                              QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                              QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic);
      plannerTabController.setAssumeFlatGroundTopic(QuadrupedUIMessagerAPI.AssumeFlatGroundTopic);
      plannerTabController.setGlobalResetTopic(QuadrupedUIMessagerAPI.GlobalResetTopic);
      plannerTabController.setPlannerPlaybackFractionTopic(QuadrupedUIMessagerAPI.PlannerPlaybackFractionTopic);
      plannerTabController.setXGaitSettingsTopic(QuadrupedUIMessagerAPI.XGaitSettingsTopic);
      plannerTabController.setShowFootstepPreviewTopic(QuadrupedUIMessagerAPI.ShowFootstepPreviewTopic);
      plannerTabController.setStepListMessageTopic(QuadrupedUIMessagerAPI.FootstepPlannerTimedStepsTopic);
   }


   public static QuadrupedUserInterface createUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                                            FootstepPlannerParameters footstepPlannerParameters,
                                                            VisibilityGraphsParameters visibilityGraphsParameters, QuadrupedXGaitSettingsReadOnly xGaitSettings)
         throws Exception
   {
      return new QuadrupedUserInterface(primaryStage, messager, modelFactory, footstepPlannerParameters, visibilityGraphsParameters, xGaitSettings);
   }
}
