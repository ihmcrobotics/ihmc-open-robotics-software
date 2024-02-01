package us.ihmc.quadrupedUI;

import java.util.function.Consumer;

import controller_msgs.msg.dds.RobotConfigurationData;
import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.SubScene;
import javafx.scene.control.Label;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.JavaFXQuadrupedVisualizer;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.ui.components.StartGoalOrientationEditor;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.PawStepPlannerMenuUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.PawStepPlannerParametersUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.PlannerReachParametersUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.QuadrupedMainTabController;
import us.ihmc.quadrupedFootstepPlanning.ui.controllers.VisibilityGraphsParametersUIController;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.BodyPawPathMeshViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.PawPathMeshViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.StartGoalPawOrientationViewer;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.StartGoalPawPositionViewer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedUI.skybox.QuadrupedSkybox3D;
import us.ihmc.quadrupedUI.uiControllers.ManualStepTabController;
import us.ihmc.quadrupedUI.uiControllers.RobotControlTabController;
import us.ihmc.quadrupedUI.uiControllers.XGaitSettingsController;
import us.ihmc.quadrupedUI.video.QuadrupedVideoViewOverlay;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;

public class QuadrupedUserInterface
{
   public static final QuadrantDependentList<Color> feetColors = new QuadrantDependentList<>(Color.web("#DA5526"), Color.web("#F68930"), Color.web("#0C695D"), Color.web("#37AFA9"));

   private final Stage primaryStage;
   private final BorderPane mainPane;
   private final JavaFXMessager messager;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPawPositionViewer startGoalPositionViewer;
   private final StartGoalPawOrientationViewer startGoalOrientationViewer;
   private final StartGoalPositionEditor startGoalPositionEditor;
   private final StartGoalOrientationEditor startGoalOrientationEditor;
   private final PawPathMeshViewer pawPathViewer;
   private final BodyPawPathMeshViewer bodyPathMeshViewer;
   private final TimeStatisticsManager timeStatisticsManager;

   private final JavaFXQuadrupedVisualizer robotVisualizer;
   private final AnimationTimer cameraTracking;
   private final Joystick joystick;
   private final AnimationTimer joystickModule;
   private final QuadrupedVideoViewOverlay videoViewOverlay;

   @FXML
   private AnchorPane sceneAnchorPane;
   @FXML
   private Label timeSinceLastUpdateLabel;
   @FXML
   private Label lastControllerTimeLabel;

   @FXML
   private PawStepPlannerMenuUIController pawStepPlannerMenuUIController;
   @FXML
   private QuadrupedMainTabController plannerTabController;
   @FXML
   private RobotControlTabController robotControlTabController;
   @FXML
   private XGaitSettingsController xGaitSettingsController;
   @FXML
   private PawStepPlannerParametersUIController pawStepPlannerParametersUIController;
   @FXML
   private PlannerReachParametersUIController plannerReachParametersUIController;
   @FXML
   private VisibilityGraphsParametersUIController visibilityGraphsParametersUIController;
   @FXML
   private ManualStepTabController manualStepTabController;

   public QuadrupedUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory, double nominalBodyHeight,
                                 PawStepPlannerParametersBasics pawPlannerParameters, VisibilityGraphsParametersBasics visibilityGraphsParameters,
                                 QuadrupedXGaitSettingsReadOnly xGaitSettings, Consumer<Graphics3DNode> graphicsMutator)
         throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      pawStepPlannerParametersUIController.setPlannerParameters(pawPlannerParameters);
      plannerReachParametersUIController.setPlannerParameters(pawPlannerParameters);
      visibilityGraphsParametersUIController.setVisbilityGraphsParameters(visibilityGraphsParameters);

      plannerTabController.attachMessager(messager);
      robotControlTabController.attachMessager(messager);
      xGaitSettingsController.attachMessager(messager, xGaitSettings);
      pawStepPlannerParametersUIController.attachMessager(messager);
      plannerReachParametersUIController.attachMessager(messager);
      visibilityGraphsParametersUIController.attachMessager(messager);
      manualStepTabController.attachMessager(messager, xGaitSettings);

      manualStepTabController.setFullRobotModelFactory(modelFactory);

      setPlannerTabTopics();
      pawStepPlannerParametersUIController.setPlannerParametersTopic(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic);
      plannerReachParametersUIController.setPlannerParametersTopic(QuadrupedUIMessagerAPI.FootstepPlannerParametersTopic);
      visibilityGraphsParametersUIController.setVisibilityGraphsParametersTopic(QuadrupedUIMessagerAPI.VisibilityGraphsParametersTopic);

      plannerTabController.bindControls();
      robotControlTabController.bindControls();
      pawStepPlannerParametersUIController.bindControls();
      plannerReachParametersUIController.bindControls();
      visibilityGraphsParametersUIController.bindControls();
      xGaitSettingsController.bindControls();
      manualStepTabController.bindControls();

      pawStepPlannerParametersUIController.loadFromFile();
      plannerReachParametersUIController.loadFromFile();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addDefaultLighting();
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(0.05, 1000000.0, true);
      SubScene subScene = view3dFactory.getSubScene();
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addNodeToView(new QuadrupedSkybox3D(subScene).getSkybox());

      Pane subScenePane = view3dFactory.getSubSceneWrappedInsidePane();

      timeStatisticsManager = new TimeStatisticsManager(timeSinceLastUpdateLabel,
                                                        lastControllerTimeLabel,
                                                        messager,
                                                        QuadrupedUIMessagerAPI.RobotConfigurationDataTopic);

      this.planarRegionViewer = new PlanarRegionViewer(messager, QuadrupedUIMessagerAPI.PlanarRegionDataTopic, QuadrupedUIMessagerAPI.ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPawPositionViewer(messager, QuadrupedUIMessagerAPI.StartPositionEditModeEnabledTopic,
                                                                    QuadrupedUIMessagerAPI.GoalPositionEditModeEnabledTopic,
                                                                    QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                                                    QuadrupedUIMessagerAPI.LowLevelGoalPositionTopic, QuadrupedUIMessagerAPI.GoalPositionTopic,
                                                                    QuadrupedUIMessagerAPI.GoalOrientationTopic, QuadrupedUIMessagerAPI.StartFeetPositionTopic,
                                                                    QuadrupedUIMessagerAPI.XGaitSettingsTopic, QuadrupedUIMessagerAPI.PlanarRegionDataTopic);
      this.startGoalOrientationViewer = new StartGoalPawOrientationViewer(messager, QuadrupedUIMessagerAPI.StartOrientationEditModeEnabledTopic,
                                                                          QuadrupedUIMessagerAPI.GoalOrientationEditModeEnabledTopic,
                                                                          QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                                                          QuadrupedUIMessagerAPI.LowLevelGoalPositionTopic,
                                                                          QuadrupedUIMessagerAPI.LowLevelGoalOrientationTopic,
                                                                          QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic);
      this.startGoalPositionEditor = new StartGoalPositionEditor(messager, subScenePane, QuadrupedUIMessagerAPI.StartPositionEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.GoalPositionEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.GoalPositionTopic,
                                                                 QuadrupedUIMessagerAPI.PlanarRegionDataTopic, QuadrupedUIMessagerAPI.SelectedRegionTopic,
                                                                 QuadrupedUIMessagerAPI.StartOrientationEditModeEnabledTopic,
                                                                 QuadrupedUIMessagerAPI.GoalOrientationEditModeEnabledTopic);
      this.startGoalOrientationEditor = new StartGoalOrientationEditor(messager, subScene, QuadrupedUIMessagerAPI.EditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.StartOrientationEditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.GoalOrientationEditModeEnabledTopic,
                                                                       QuadrupedUIMessagerAPI.StartPositionTopic, QuadrupedUIMessagerAPI.StartOrientationTopic,
                                                                       QuadrupedUIMessagerAPI.GoalPositionTopic, QuadrupedUIMessagerAPI.GoalOrientationTopic,
                                                                       QuadrupedUIMessagerAPI.SelectedRegionTopic);
      this.pawPathViewer = new PawPathMeshViewer(messager, QuadrupedUIMessagerAPI.FootstepPlanTopic, QuadrupedUIMessagerAPI.ComputePathTopic,
                                                 QuadrupedUIMessagerAPI.ShowFootstepPlanTopic, QuadrupedUIMessagerAPI.ShowFootstepPreviewTopic);
      pawPathViewer.setFootstepRadius(0.025);
      pawPathViewer.setFootstepColors(feetColors);

      this.bodyPathMeshViewer = new BodyPawPathMeshViewer(messager, QuadrupedUIMessagerAPI.ShowBodyPathTopic, QuadrupedUIMessagerAPI.ComputePathTopic,
                                                          QuadrupedUIMessagerAPI.BodyPathDataTopic);

      plannerTabController.setPreviewFootstepPositions(pawPathViewer.getPreviewFootstepPositions());

      manualStepTabController.initScene(subScene);
      primaryStage.addEventHandler(KeyEvent.ANY, this::onKeyEvent);

      robotVisualizer = new JavaFXQuadrupedVisualizer(modelFactory, graphicsMutator);
      robotVisualizer.attachMessager(messager, QuadrupedUIMessagerAPI.RobotModelTopic);
      messager.addTopicListener(QuadrupedUIMessagerAPI.RobotConfigurationDataTopic, this::submitNewConfiguration);

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
         joystickModule = new QuadrupedJoystickModule(messager, xGaitSettings, robotVisualizer.getFullRobotModel(), nominalBodyHeight, joystick);
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
      videoViewOverlay = new QuadrupedVideoViewOverlay(width, height, true, true);
      sceneAnchorPane.getChildren().set(1, videoViewOverlay.getNode());
      AnchorPane.setTopAnchor(videoViewOverlay.getNode(), 0.0);
      AnchorPane.setLeftAnchor(videoViewOverlay.getNode(), 0.0);
      videoViewOverlay.getNode().addEventHandler(MouseEvent.MOUSE_PRESSED, event -> videoViewOverlay.toggleMode());

      videoViewOverlay.start(messager, QuadrupedUIMessagerAPI.LeftCameraVideo);
      timeStatisticsManager.start();
      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalPositionEditor.start();
      startGoalOrientationEditor.start();
      pawPathViewer.start();
      bodyPathMeshViewer.start();

      robotVisualizer.start();
      cameraTracking.start();

      sceneAnchorPane.getChildren().set(0, subScenePane);
      AnchorPane.setTopAnchor(subScenePane, 0.0);
      AnchorPane.setBottomAnchor(subScenePane, 0.0);
      AnchorPane.setLeftAnchor(subScenePane, 0.0);
      AnchorPane.setRightAnchor(subScenePane, 0.0);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }


   private void onKeyEvent(KeyEvent keyEvent)
   {
      // pressed and released only use code field
      if (keyEvent.getEventType() == KeyEvent.KEY_PRESSED && keyEvent.getCode() == KeyCode.ESCAPE)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
         messager.submitMessage(QuadrupedUIMessagerAPI.AbortWalkingTopic, true);
      }
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
      timeStatisticsManager.stop();
      plannerTabController.stop();
      planarRegionViewer.stop();
      startGoalPositionViewer.stop();
      startGoalOrientationViewer.stop();
      startGoalPositionEditor.stop();
      startGoalOrientationEditor.stop();
      pawPathViewer.stop();
      bodyPathMeshViewer.stop();
      cameraTracking.stop();
      videoViewOverlay.stop();
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
      plannerTabController.setPlanarRegionDataClearTopic(QuadrupedUIMessagerAPI.PlanarRegionDataClearTopic);
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
      plannerTabController.setDesiredSteppingStateNameTopic(QuadrupedUIMessagerAPI.DesiredSteppingStateNameTopic, QuadrupedUIMessagerAPI.CurrentSteppingStateNameTopic);
      plannerTabController.setAbortWalkingTopic(QuadrupedUIMessagerAPI.AbortWalkingTopic);
      plannerTabController.setEnableStepTeleopTopic(QuadrupedUIMessagerAPI.EnableStepTeleopTopic);
   }

   public static QuadrupedUserInterface createUserInterface(Stage primaryStage, JavaFXMessager messager, QuadrupedModelFactory modelFactory,
                                                            PawStepPlannerParametersBasics pawPlannerParameters,
                                                            VisibilityGraphsParametersBasics visibilityGraphsParameters, double nominalBodyHeight,
                                                            QuadrupedXGaitSettingsReadOnly xGaitSettings, Consumer<Graphics3DNode> graphicsMutator) throws Exception
   {
      return new QuadrupedUserInterface(primaryStage, messager, modelFactory, nominalBodyHeight, pawPlannerParameters, visibilityGraphsParameters,
                                        xGaitSettings, graphicsMutator);
   }
}
