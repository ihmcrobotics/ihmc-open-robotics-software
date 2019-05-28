package us.ihmc.quadrupedUI;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.AnchorPane;
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
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.*;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.BodyPathMeshViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.FootstepPathMeshViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.StartGoalOrientationViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.StartGoalPositionViewer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedUI.uiControllers.RobotControlTabController;
import us.ihmc.quadrupedUI.uiControllers.ManualStepTabController;
import us.ihmc.quadrupedUI.uiControllers.XGaitSettingsController;
import us.ihmc.quadrupedUI.video.JavaFXROS2VideoView;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;

public class QuadrupedUserInterface
{
   private final Stage primaryStage;
   private final SplitPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final StartGoalOrientationViewer startGoalOrientationViewer;
   private final StartGoalPositionEditor startGoalPositionEditor;
   private final StartGoalOrientationEditor startGoalOrientationEditor;
   private final FootstepPathMeshViewer pawPathViewer;
   private final BodyPathMeshViewer bodyPathMeshViewer;


   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final AnimationTimer cameraTracking;
   private final Joystick joystick;
   private final AnimationTimer joystickModule;
   private final JavaFXROS2VideoView videoView;

   @FXML
   private SplitPane mainViewSplitPane;
   @FXML
   private AnchorPane firstPersonViewPane;

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
   private PlannerReachParametersUIController plannerReachParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private ManualStepTabController manualStepTabController;

   public QuadrupedUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory, double nominalBodyHeight,
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
      plannerReachParametersUIController.setPlannerParameters(footstepPlannerParameters);
      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);

      plannerTabController.attachMessager(messager);
      robotControlTabController.attachMessager(messager);
      xGaitSettingsController.attachMessager(messager, xGaitSettings);
      footstepPlannerParametersUIController.attachMessager(messager);
      plannerReachParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      manualStepTabController.attachMessager(messager, xGaitSettings);

      manualStepTabController.setFullRobotModelFactory(modelFactory);

      setPlannerTabTopics();
      footstepPlannerParametersUIController.setPlannerParametersTopic(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic);
      plannerReachParametersUIController.setPlannerParametersTopic(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic);
      visibilityGraphsParametersUIController.setVisibilityGraphsParametersTopic(QuadrupedUIMessagerAPI.VisibilityGraphsParametersTopic);

      plannerTabController.bindControls();
      robotControlTabController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      plannerReachParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      xGaitSettingsController.bindControls();
      manualStepTabController.bindControls();

      footstepPlannerParametersUIController.loadFromFile();
      plannerReachParametersUIController.loadFromFile();

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

      plannerTabController.setPreviewFootstepPositions(pawPathViewer.getPreviewFootstepPositions());

      manualStepTabController.initScene(view3dFactory.getSubScene());

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
      view3dFactory.addNodeToView(manualStepTabController);

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

      if (Joystick.isAJoystickConnectedToSystem())
      {
         joystick = new Joystick(JoystickModel.XBOX_ONE, 0);
         joystickModule = new QuadrupedJoystickModule(messager, xGaitSettings, nominalBodyHeight, joystick);
         joystickModule.start();
      }
      else
      {
         joystick = null;
         joystickModule = null;
         LogTools.warn("No joystick detected, running without xbox module");
      }

      int width = 1024;
      int height = 544;
      videoView = new JavaFXROS2VideoView(width, height, false, true);
      firstPersonViewPane.getChildren().add(videoView);
//      AnchorPane.setLeftAnchor(videoView, 0.0);
//      AnchorPane.setTopAnchor(videoView, 0.0);

      videoView.start(messager, QuadrupedUIMessagerAPI.LeftCameraVideo);
      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalPositionEditor.start();
      startGoalOrientationEditor.start();
      pawPathViewer.start();
      bodyPathMeshViewer.start();

      robotVisualizer.start();
      cameraTracking.start();

      mainViewSplitPane.getItems().set(1, subScene);
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
      videoView.stop();
      manualStepTabController.stop();

      if (joystick != null)
         joystick.shutdown();
      if(joystickModule != null)
         joystickModule.stop();

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
                                              QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic,
                                              QuadrupedUIMessagerAPI.StartTargetTypeTopic, QuadrupedUIMessagerAPI.StartFeetPositionTopic);
      plannerTabController.setAssumeFlatGroundTopic(QuadrupedUIMessagerAPI.AssumeFlatGroundTopic);
      plannerTabController.setGlobalResetTopic(QuadrupedUIMessagerAPI.GlobalResetTopic);
      plannerTabController.setPlannerPlaybackFractionTopic(QuadrupedUIMessagerAPI.PlannerPlaybackFractionTopic);
      plannerTabController.setXGaitSettingsTopic(QuadrupedUIMessagerAPI.XGaitSettingsTopic);
      plannerTabController.setShowFootstepPreviewTopic(QuadrupedUIMessagerAPI.ShowFootstepPreviewTopic);
      plannerTabController.setStepListMessageTopic(QuadrupedUIMessagerAPI.FootstepPlannerTimedStepsTopic);
      plannerTabController.setDesiredSteppingStateNameTopic(QuadrupedUIMessagerAPI.DesiredSteppingStateNameTopic);
   }

   public static QuadrupedUserInterface createUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                                            FootstepPlannerParameters footstepPlannerParameters,
                                                            VisibilityGraphsParameters visibilityGraphsParameters, double nominalBodyHeight,
                                                            QuadrupedXGaitSettingsReadOnly xGaitSettings) throws Exception
   {
      return new QuadrupedUserInterface(primaryStage, messager, modelFactory, nominalBodyHeight, footstepPlannerParameters, visibilityGraphsParameters,
                                        xGaitSettings);
   }
}
