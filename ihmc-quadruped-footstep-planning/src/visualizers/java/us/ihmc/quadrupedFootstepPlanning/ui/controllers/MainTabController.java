package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.QuadrupedTimedStepListMessage;
import controller_msgs.msg.dds.QuadrupedTimedStepMessage;
import javafx.animation.AnimationTimer;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.TextField;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.TopicListener;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.YawProperty;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerStatus;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerTargetType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.geometry.GroundPlaneEstimator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;

public class MainTabController
{
   private static final boolean verbose = true;

   // control
   @FXML
   private ComboBox<PawStepPlannerType> plannerType;
   @FXML
   private CheckBox acceptNewRegions;
   @FXML
   private CheckBox isPlanAdjustable;
   @FXML
   private CheckBox correctStepHeightError;
   @FXML
   private CheckBox assumeFlatGround;
   @FXML
   private Spinner<Double> timeout;
   @FXML
   private Spinner<Double> horizonLength;

   // status
   @FXML
   private TextField sentRequestId;
   @FXML
   private TextField receivedRequestId;

   @FXML
   private TextField timeTaken;
   @FXML
   private TextField planningResult;
   @FXML
   private TextField plannerStatus;

   // goal placement
   @FXML
   private Button placeStart;
   @FXML
   private Button placeGoal;

   @FXML
   private Button computePath;
   @FXML
   private Button abortPlanning;

   @FXML
   private Spinner<Double> startXPosition;
   @FXML
   private Spinner<Double> startYPosition;
   @FXML
   private Spinner<Double> startZPosition;
   @FXML
   private Spinner<Double> goalXPosition;
   @FXML
   private Spinner<Double> goalYPosition;
   @FXML
   private Spinner<Double> goalZPosition;
   @FXML
   private ComboBox<RobotQuadrant> initialSupportQuadrant;

   @FXML
   private Spinner<Double> startYaw;
   @FXML
   private Spinner<Double> goalYaw;

   @FXML
   private Slider previewSlider;
   @FXML
   private Button sendPlanButton;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   @FXML
   public void computePath()
   {
      if (verbose)
         LogTools.info("Clicked compute path...");

      if (quadrupedReferenceFrames != null)
         setStartFromRobot();

      int newRequestID = currentPlannerRequestId.get() + 1;
      messager.submitMessage(plannerRequestIdTopic, newRequestID);
      messager.submitMessage(computePathTopic, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         LogTools.info("Clicked abort planning...");

      messager.submitMessage(abortPlanningTopic, true);
   }

   @FXML
   public void sendPlan()
   {
      PawStepPlan pawStepPlan = footstepPlanReference.get();
      if (pawStepPlan == null)
         return;

      QuadrupedTimedStepListMessage stepMessages = new QuadrupedTimedStepListMessage();
      for (int i = 0; i < pawStepPlan.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStepMessage stepMessage = stepMessages.getQuadrupedStepList().add();
         QuadrupedTimedStep step = pawStepPlan.getPawStep(i);

         stepMessage.getQuadrupedStepMessage().setRobotQuadrant(step.getRobotQuadrant().toByte());
         stepMessage.getQuadrupedStepMessage().getGoalPosition().set(step.getGoalPosition());
         stepMessage.getQuadrupedStepMessage().setGroundClearance(step.getGroundClearance());
         stepMessage.getTimeInterval().setStartTime(step.getTimeInterval().getStartTime());
         stepMessage.getTimeInterval().setEndTime(step.getTimeInterval().getEndTime());
      }

      stepMessages.setIsExpressedInAbsoluteTime(false);
      stepMessages.setAreStepsAdjustable(isPlanAdjustable.isSelected());
      stepMessages.setOffsetStepsHeightWithExecutionError(correctStepHeightError.isSelected());

      if (verbose)
         LogTools.info("Sending step list...");
      messager.submitMessage(stepListMessageTopic, stepMessages);
   }

   @FXML
   public void requestStopWalking()
   {
      if (abortWalkingTopic != null)
      {
         messager.submitMessage(abortWalkingTopic, true);
      }

      if (enableStepTeleopTopic != null)
      {
         messager.submitMessage(enableStepTeleopTopic, false);
      }
   }

   @FXML
   public void clearFootstepPlan()
   {
      messager.submitMessage(footstepPlanTopic, null);
   }

   @FXML
   public void clearFlat()
   {
      acceptNewRegions.setSelected(false);
      assumeFlatGround.setSelected(true);
      messager.submitMessage(planarRegionDataTopic, buildFlatGround());
   }

   @FXML
   public void requestClearREA()
   {
      messager.submitMessage(planarRegionDataClearTopic, true);
   }

   private PlanarRegionsList buildFlatGround()
   {
      quadrupedReferenceFrames.updateFrames();

      GroundPlaneEstimator groundPlaneEstimator = new GroundPlaneEstimator();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D groundPlanePosition = new FramePoint3D(quadrupedReferenceFrames.getSoleFrame(robotQuadrant));
         groundPlanePosition.changeFrame(ReferenceFrame.getWorldFrame());
         groundPlaneEstimator.addContactPoint(groundPlanePosition);
      }
      groundPlaneEstimator.compute();

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(10.0, 10.0);
      convexPolygon.addVertex(-10.0, 10.0);
      convexPolygon.addVertex(-10.0, -10.0);
      convexPolygon.addVertex(10.0, -10.0);
      convexPolygon.update();
      PlanarRegion groundPlane = new PlanarRegion(groundPlaneEstimator.getGroundPlaneFrame().getTransformToWorldFrame(), convexPolygon);
      return new PlanarRegionsList(groundPlane);
   }

   private JavaFXMessager messager;

   private AtomicReference<Integer> currentPlannerRequestId;
   private FootstepPlanPreviewPlaybackManager footstepPlanPreviewPlaybackManager;
   private QuadrupedReferenceFrames quadrupedReferenceFrames;
   private AtomicReference<PawStepPlan> footstepPlanReference;
   private AtomicReference<Point3D> startPositionReference;
   private AtomicReference<Quaternion> startOrientationReference;

   private final Point3DProperty startPositionProperty = new Point3DProperty(this, "startPositionProperty", new Point3D());
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());

   private final YawProperty startRotationProperty = new YawProperty(this, "startRotationProperty", 0.0);
   private final YawProperty goalRotationProperty = new YawProperty(this, "goalRotationProperty", 0.0);

   private Topic<PawStepPlannerType> plannerTypeTopic;
   private Topic<Integer> plannerRequestIdTopic;
   private Topic<Integer> receivedPlanIdTopic;
   private Topic<Boolean> showFootstepPlanTopic;
   private Topic<PawStepPlan> footstepPlanTopic;
   private Topic<Boolean> planarRegionDataClearTopic;
   private Topic<PlanarRegionsList> planarRegionDataTopic;
   private Topic<Double> plannerTimeTakenTopic;
   private Topic<Double> plannerTimeoutTopic;
   private Topic<Boolean> computePathTopic;
   private Topic<Boolean> abortPlanningTopic;
   private Topic<Boolean> acceptNewPlanarRegionsTopic;
   private Topic<PawStepPlanningResult> planningResultTopic;
   private Topic<PawStepPlannerStatus> plannerStatusTopic;
   private Topic<Double> plannerHorizonLengthTopic;
   private Topic<Boolean> editModeEnabledTopic;
   private Topic<Boolean> startPositionEditModeEnabledTopic;
   private Topic<Boolean> goalPositionEditModeEnabledTopic;
   private Topic<RobotQuadrant> initialSupportQuadrantTopic;
   private Topic<Point3D> startPositionTopic;
   private Topic<PawStepPlannerTargetType> startTargetTypeTopic;
   private Topic<QuadrantDependentList<Point3D>> startFeetPositionTopic;
   private Topic<Quaternion> startOrientationTopic;
   private Topic<Point3D> goalPositionTopic;
   private Topic<Quaternion> goalOrientationTopic;
   private Topic<Boolean> assumeFlatGroundTopic;
   private Topic<Boolean> globalResetTopic;
   private Topic<Number> plannerPlaybackFractionTopic;
   private Topic<QuadrupedXGaitSettingsReadOnly> xGaitSettingsTopic;
   private Topic<Boolean> showFootstepPreviewTopic;
   private Topic<QuadrupedTimedStepListMessage> stepListMessageTopic;
   private Topic<QuadrupedSteppingStateEnum> desiredSteppingStateNameTopic;
   private Topic<QuadrupedSteppingStateEnum> currentSteppingStateNameTopic;
   private Topic<Boolean> abortWalkingTopic;
   private Topic<Boolean> enableStepTeleopTopic;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerTypeTopic(Topic<PawStepPlannerType> plannerTypeTopic)
   {
      this.plannerTypeTopic = plannerTypeTopic;
   }

   public void setPlannerRequestIdTopic(Topic<Integer> plannerRequestIdTopic)
   {
      this.plannerRequestIdTopic = plannerRequestIdTopic;
   }

   public void setReceivedPlanIdTopic(Topic<Integer> receivedPlanIdTopic)
   {
      this.receivedPlanIdTopic = receivedPlanIdTopic;
   }

   public void setFootstepPlanTopic(Topic<Boolean> showFootstepPlanTopic, Topic<PawStepPlan> footstepPlanTopic)
   {
      this.showFootstepPlanTopic = showFootstepPlanTopic;
      this.footstepPlanTopic = footstepPlanTopic;
   }

   public void setPlanarRegionDataClearTopic(Topic<Boolean> planarRegionDataClearTopic)
   {
      this.planarRegionDataClearTopic = planarRegionDataClearTopic;
   }

   public void setAbortWalkingTopic(Topic<Boolean> abortWalkingTopic)
   {
      this.abortWalkingTopic = abortWalkingTopic;
   }

   public void setEnableStepTeleopTopic(Topic<Boolean> enableStepTeleopTopic)
   {
      this.enableStepTeleopTopic = enableStepTeleopTopic;
   }

   public void setPlanarRegionDataTopic(Topic<PlanarRegionsList> planarRegionDataTopic)
   {
      this.planarRegionDataTopic = planarRegionDataTopic;
   }

   public void setPlannerTimeTakenTopic(Topic<Double> plannerTimeTakenTopic)
   {
      this.plannerTimeTakenTopic = plannerTimeTakenTopic;
   }

   public void setPlannerTimeoutTopic(Topic<Double> plannerTimeoutTopic)
   {
      this.plannerTimeoutTopic = plannerTimeoutTopic;
   }

   public void setComputePathTopic(Topic<Boolean> computePathTopic)
   {
      this.computePathTopic = computePathTopic;
   }

   public void setAbortPlanningTopic(Topic<Boolean> abortPlanningTopic)
   {
      this.abortPlanningTopic = abortPlanningTopic;
   }

   public void setAcceptNewPlanarRegionsTopic(Topic<Boolean> acceptNewPlanarRegionsTopic)
   {
      this.acceptNewPlanarRegionsTopic = acceptNewPlanarRegionsTopic;
   }

   public void setPlanningResultTopic(Topic<PawStepPlanningResult> planningResultTopic)
   {
      this.planningResultTopic = planningResultTopic;
   }

   public void setPlannerStatusTopic(Topic<PawStepPlannerStatus> plannerStatusTopic)
   {
      this.plannerStatusTopic = plannerStatusTopic;
   }

   public void setPlannerHorizonLengthTopic(Topic<Double> plannerHorizonLengthTopic)
   {
      this.plannerHorizonLengthTopic = plannerHorizonLengthTopic;
   }

   public void setStartGoalTopics(Topic<Boolean> editModeEnabledTopic, Topic<Boolean> startPositionEditModeEnabledTopic,
                                  Topic<Boolean> goalPositionEditModeEnabledTopic, Topic<RobotQuadrant> initialSupportQuadrantTopic,
                                  Topic<Point3D> startPositionTopic, Topic<Quaternion> startOrientationTopic, Topic<Point3D> goalPositionTopic,
                                  Topic<Quaternion> goalOrientationTopic, Topic<PawStepPlannerTargetType> startTargetTypeTopic,
                                  Topic<QuadrantDependentList<Point3D>> startFeetPositionTopic)
   {
      this.editModeEnabledTopic = editModeEnabledTopic;
      this.startPositionEditModeEnabledTopic = startPositionEditModeEnabledTopic;
      this.goalPositionEditModeEnabledTopic = goalPositionEditModeEnabledTopic;
      this.initialSupportQuadrantTopic = initialSupportQuadrantTopic;
      this.startPositionTopic = startPositionTopic;
      this.startOrientationTopic = startOrientationTopic;
      this.goalPositionTopic = goalPositionTopic;
      this.goalOrientationTopic = goalOrientationTopic;
      this.startTargetTypeTopic = startTargetTypeTopic;
      this.startFeetPositionTopic = startFeetPositionTopic;
   }

   public void setAssumeFlatGroundTopic(Topic<Boolean> assumeFlatGroundTopic)
   {
      this.assumeFlatGroundTopic = assumeFlatGroundTopic;
   }

   public void setGlobalResetTopic(Topic<Boolean> globalResetTopic)
   {
      this.globalResetTopic = globalResetTopic;
   }

   public void setPlannerPlaybackFractionTopic(Topic<Number> plannerPlaybackFractionTopic)
   {
      this.plannerPlaybackFractionTopic = plannerPlaybackFractionTopic;
   }

   public void setXGaitSettingsTopic(Topic<QuadrupedXGaitSettingsReadOnly> xGaitSettingsTopic)
   {
      this.xGaitSettingsTopic = xGaitSettingsTopic;
   }

   public void setShowFootstepPreviewTopic(Topic<Boolean> showFootstepPreviewTopic)
   {
      this.showFootstepPreviewTopic = showFootstepPreviewTopic;
   }

   public void setStepListMessageTopic(Topic<QuadrupedTimedStepListMessage> stepListMessageTopic)
   {
      this.stepListMessageTopic = stepListMessageTopic;
   }

   public void setDesiredSteppingStateNameTopic(Topic<QuadrupedSteppingStateEnum> desiredSteppingStateNameTopic, Topic<QuadrupedSteppingStateEnum> currentSteppingStateNameTopic)
   {
      this.desiredSteppingStateNameTopic = desiredSteppingStateNameTopic;
      this.currentSteppingStateNameTopic = currentSteppingStateNameTopic;
   }

   public void bindControls()
   {
      setupControls();

      currentPlannerRequestId = messager.createInput(plannerRequestIdTopic, -1);
      footstepPlanReference = messager.createInput(footstepPlanTopic, null);

      startPositionReference = messager.createInput(startPositionTopic);
      startOrientationReference = messager.createInput(startOrientationTopic);

      // control
      messager.bindBidirectional(plannerTypeTopic, plannerType.valueProperty(), true);
      messager.registerJavaFXSyncedTopicListener(plannerRequestIdTopic, new TextViewerListener<>(sentRequestId));
      messager.registerJavaFXSyncedTopicListener(receivedPlanIdTopic, new TextViewerListener<>(receivedRequestId));
      messager.registerJavaFXSyncedTopicListener(plannerTimeTakenTopic, new TextViewerListener<>(timeTaken));
      messager.registerJavaFXSyncedTopicListener(planningResultTopic, new TextViewerListener<>(planningResult));
      messager.registerJavaFXSyncedTopicListener(plannerStatusTopic, new TextViewerListener<>(plannerStatus));

      messager.bindBidirectional(acceptNewPlanarRegionsTopic, acceptNewRegions.selectedProperty(), true);

      messager.bindBidirectional(plannerTimeoutTopic, timeout.getValueFactory().valueProperty(), doubleToDoubleConverter, true);

      messager.bindBidirectional(plannerHorizonLengthTopic, horizonLength.getValueFactory().valueProperty(), doubleToDoubleConverter,
                                 true);

      // set goal
      messager.bindPropertyToTopic(editModeEnabledTopic, placeStart.disableProperty());
      messager.bindPropertyToTopic(editModeEnabledTopic, placeGoal.disableProperty());
      messager.bindPropertyToTopic(editModeEnabledTopic, computePath.disableProperty());
      messager.bindPropertyToTopic(editModeEnabledTopic, abortPlanning.disableProperty());

      messager.bindBidirectional(assumeFlatGroundTopic, assumeFlatGround.selectedProperty(), false);

      messager.bindBidirectional(initialSupportQuadrantTopic, initialSupportQuadrant.valueProperty(), true);

      startPositionProperty.bindBidirectionalX(startXPosition.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYPosition.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZPosition.getValueFactory().valueProperty());
      messager.bindBidirectional(startPositionTopic, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZPosition.getValueFactory().valueProperty());
      messager.bindBidirectional(goalPositionTopic, goalPositionProperty, false);

      startRotationProperty.bindBidirectionalYaw(startYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(startOrientationTopic, startRotationProperty, false);

      goalRotationProperty.bindBidirectionalYaw(goalYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(goalOrientationTopic, goalRotationProperty, false);

      messager.registerTopicListener(globalResetTopic, reset -> clearStartGoalTextFields());

      messager.bindBidirectional(plannerPlaybackFractionTopic, previewSlider.valueProperty(), false);

      //      walkingPreviewPlaybackManager = new WalkingPreviewPlaybackManager(messager);
      footstepPlanPreviewPlaybackManager = new FootstepPlanPreviewPlaybackManager(messager);
      previewSlider.valueProperty()
                   .addListener((observable, oldValue, newValue) -> footstepPlanPreviewPlaybackManager.requestSpecificPercentageInPreview(newValue.doubleValue()));

      messager.registerTopicListener(footstepPlanTopic, plan -> sendPlanButton.setDisable(plan == null || !isValidPlan(plan)));
      if (abortWalkingTopic != null)
         messager.registerJavaFXSyncedTopicListener(abortWalkingTopic, m -> clearFootstepPlan());
      if (currentSteppingStateNameTopic != null)
      {
         messager.registerJavaFXSyncedTopicListener(currentSteppingStateNameTopic, m -> {
            if (m != null && m == QuadrupedSteppingStateEnum.STAND)
               clearFootstepPlan();
         });
      }
   }

   private boolean isValidPlan(PawStepPlan plan)
   {
      if (quadrupedReferenceFrames == null)
         return true;
      double maximumStepTranslation = 1.0;
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         FramePoint3D startPosition = new FramePoint3D();
         Point3DReadOnly goalPosition = plan.getPawStep(i).getGoalPosition();

         if(i < 4)
         {
            startPosition.setToZero(quadrupedReferenceFrames.getSoleFrame(plan.getPawStep(i).getRobotQuadrant()));
            startPosition.changeFrame(ReferenceFrame.getWorldFrame());
         }
         else
         {
            QuadrupedTimedStep previousStep = plan.getPawStep(i - 4);
            startPosition.set(previousStep.getGoalPosition());
         }

         if(startPosition.distance(goalPosition) > maximumStepTranslation)
         {
            return false;
         }
      }

      return true;
   }

   private void setupControls()
   {
      startXPosition.setValueFactory(createStartGoalPositionValueFactory());
      startYPosition.setValueFactory(createStartGoalPositionValueFactory());
      startZPosition.setValueFactory(createStartGoalPositionValueFactory());

      goalXPosition.setValueFactory(createStartGoalPositionValueFactory());
      goalYPosition.setValueFactory(createStartGoalPositionValueFactory());
      goalZPosition.setValueFactory(createStartGoalPositionValueFactory());

      startYaw.setValueFactory(createStartGoalOrientationValueFactory());
      goalYaw.setValueFactory(createStartGoalOrientationValueFactory());


      ObservableList<PawStepPlannerType> plannerTypeOptions = FXCollections.observableArrayList(PawStepPlannerType.values);
      plannerType.setItems(plannerTypeOptions);
      plannerType.setValue(PawStepPlannerType.A_STAR);

      timeout.setValueFactory(createTimeoutValueFactory());
      horizonLength.setValueFactory(createHorizonValueFactory());

      initialSupportQuadrant.setItems(FXCollections.observableArrayList(RobotQuadrant.values));
      initialSupportQuadrant.setValue(RobotQuadrant.FRONT_LEFT);
   }

   @FXML
   public void placeStart()
   {
      messager.submitMessage(startPositionEditModeEnabledTopic, true);
      messager.submitMessage(editModeEnabledTopic, true);
   }

   @FXML
   public void placeGoal()
   {
      messager.submitMessage(goalPositionEditModeEnabledTopic, true);
      messager.submitMessage(editModeEnabledTopic, true);
   }

   private void setStartFromRobot()
   {
      quadrupedReferenceFrames.updateFrames();
      FramePose3D startPose = new FramePose3D(quadrupedReferenceFrames.getBodyZUpFrame());
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      startPositionProperty.set(new Point3D(startPose.getPosition()));
      startRotationProperty.set(new Quaternion(startPose.getYaw(), 0.0, 0.0));

      QuadrantDependentList<Point3D> startFeetPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D(quadrupedReferenceFrames.getSoleFrame(robotQuadrant));
         footPosition.changeFrame(ReferenceFrame.getWorldFrame());
         startFeetPositions.put(robotQuadrant, new Point3D(footPosition));
      }

      messager.submitMessage(startTargetTypeTopic, PawStepPlannerTargetType.FOOTSTEPS);
      messager.submitMessage(startFeetPositionTopic, startFeetPositions);
   }

   /*
   @FXML
   private void requestWalkingPreview()
   {
      WalkingControllerPreviewInputMessage requestMessage = new WalkingControllerPreviewInputMessage();
      requestMessage.setSequenceId(walkingPreviewRequestId.incrementAndGet());

      FootstepPlan footstepPlan = footstepPlanReference.get();
      if (footstepPlan == null)
         return;

      double swingTime = 1.2;
      double transferTime = 1.0;
      throw new RuntimeException("This feature is not yet implemented.");
//      requestMessage.footsteps_.set(FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, swingTime, transferTime, ExecutionMode.OVERRIDE));
//      messager.submitMessage(FootstepPlannerMessagerAPI.RequestWalkingPreview, requestMessage);
   }
   */

   @FXML
   private void playFootstepPlanPreview()
   {
      footstepPlanPreviewPlaybackManager.start();
      messager.submitMessage(showFootstepPreviewTopic, true);
   }

   @FXML
   private void pauseFootstepPlanPreview()
   {
      footstepPlanPreviewPlaybackManager.playbackModeActive.set(false);
   }

   @FXML
   private void stopFootstepPlanPreview()
   {
      footstepPlanPreviewPlaybackManager.stop();
      messager.submitMessage(showFootstepPreviewTopic, false);
      messager.submitMessage(showFootstepPlanTopic, true);
   }


   public void setFullRobotModel(FullQuadrupedRobotModel fullQuadrupedRobotModel)
   {
      this.quadrupedReferenceFrames = new QuadrupedReferenceFrames(fullQuadrupedRobotModel);
   }

   public void setPreviewModel(FullQuadrupedRobotModel previewRobotModel)
   {
//      this.walkingPreviewPlaybackManager.setRobotModel(previewRobotModel);
   }

   public void setPreviewFootstepPositions(QuadrantDependentList<Point3D> footstepPositions)
   {
      footstepPlanPreviewPlaybackManager.setPreviewFootstepPositions(footstepPositions);
   }


   private void clearStartGoalTextFields()
   {
      startXPosition.valueFactoryProperty().getValue().setValue(0.0);
      startYPosition.valueFactoryProperty().getValue().setValue(0.0);
      startZPosition.valueFactoryProperty().getValue().setValue(0.0);

      goalXPosition.valueFactoryProperty().getValue().setValue(0.0);
      goalYPosition.valueFactoryProperty().getValue().setValue(0.0);
      goalZPosition.valueFactoryProperty().getValue().setValue(0.0);

      startYaw.valueFactoryProperty().getValue().setValue(0.0);
      goalYaw.valueFactoryProperty().getValue().setValue(0.0);
   }

   private DoubleSpinnerValueFactory createStartGoalPositionValueFactory()
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private DoubleSpinnerValueFactory createStartGoalOrientationValueFactory()
   {
      double min = -Math.PI;
      double max = Math.PI;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private DoubleSpinnerValueFactory createTimeoutValueFactory()
   {
      double min = 0.0;
      double max = 500.0;
      double amountToStepBy = 5;
      return new DoubleSpinnerValueFactory(min, max, 15.0, amountToStepBy);
   }

   private DoubleSpinnerValueFactory createHorizonValueFactory()
   {
      double min = 0.0;
      double max = 1000.0;
      double amountToStepBy = 0.25;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private final PropertyToMessageTypeConverter<Double, Double> doubleToDoubleConverter = new PropertyToMessageTypeConverter<Double, Double>()
   {
      @Override
      public Double convert(Double propertyValue)
      {
         return propertyValue;
      }

      @Override
      public Double interpret(Double newValue)
      {
         return newValue;
      }
   };

   private class TextViewerListener<T> implements TopicListener<T>
   {
      private final TextField textField;

      public TextViewerListener(TextField textField)
      {
         this.textField = textField;
      }

      @Override
      public void receivedMessageForTopic(T messageContent)
      {
         if (messageContent != null)
            textField.promptTextProperty().setValue(messageContent.toString());
      }
   }

   private class FootstepPlanPreviewPlaybackManager extends AnimationTimer
   {
      private final AtomicReference<QuadrupedXGaitSettingsReadOnly> xGaitSettingsReference;

      private final double frameDt = 0.005;
      // frames per call to handle()
      final int playbackSpeed = 50;
      int playbackCounter = 0;

      // whether to show ghost robot
      final AtomicBoolean active = new AtomicBoolean(false);

      // whether to animate ghost robot
      final AtomicBoolean playbackModeActive = new AtomicBoolean(false);
      private final List<QuadrantDependentList<Point3DReadOnly>> footPositionScene = new ArrayList<>();

      private QuadrantDependentList<Point3D> previewFootstepPositions;

      FootstepPlanPreviewPlaybackManager(Messager messager)
      {
         xGaitSettingsReference = messager.createInput(xGaitSettingsTopic);

         messager.registerTopicListener(footstepPlanTopic, footstepPlan -> executorService.submit(() -> {
           calculateFrames(footstepPlan);
         }));
      }

      void setPreviewFootstepPositions(QuadrantDependentList<Point3D> previewFootstepPositions)
      {
         this.previewFootstepPositions = previewFootstepPositions;
      }

      private void calculateFrames(PawStepPlan pawStepPlan)
      {
         footPositionScene.clear();

         if (pawStepPlan == null || pawStepPlan.getNumberOfSteps() < 1)
            return;

         List<QuadrupedTimedStep> steps = new ArrayList<>();
         for (int i = 0; i < pawStepPlan.getNumberOfSteps(); i++)
            steps.add(pawStepPlan.getPawStep(i));
         TimeIntervalTools.sortByEndTime(steps);
         double endTime = steps.get(pawStepPlan.getNumberOfSteps() - 1).getTimeInterval().getEndTime();
         TimeIntervalTools.sortByStartTime(steps);
         double startTime = steps.get(0).getTimeInterval().getStartTime() - 1.0;


         PoseReferenceFrame xGaitFrame = new PoseReferenceFrame("xGaitFrame", ReferenceFrame.getWorldFrame());
         xGaitFrame.setPoseAndUpdate(startPositionReference.get(), startOrientationReference.get());



         double xOffset = 0.5 * xGaitSettingsReference.get().getStanceLength();
         double yOffset = 0.5 * xGaitSettingsReference.get().getStanceWidth();

         QuadrantDependentList<Point3DReadOnly> currentFootstepPositions = new QuadrantDependentList<>();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            FramePoint3D footPosition = new FramePoint3D(xGaitFrame, robotQuadrant.getEnd().negateIfHindEnd(xOffset),
                                                         robotQuadrant.getSide().negateIfRightSide(yOffset), 0.0);
            footPosition.changeFrame(ReferenceFrame.getWorldFrame());
            currentFootstepPositions.put(robotQuadrant, footPosition);
         }

         footPositionScene.add(currentFootstepPositions);

         double time = startTime;
         while (time <= endTime)
         {
            // copy the previous positions
            currentFootstepPositions = new QuadrantDependentList<>(currentFootstepPositions);

            // handle steps just completed
            List<QuadrupedTimedStep> stepsFinished = TimeIntervalTools.removeAndReturnEndTimesLessThan(time, steps);
            for (QuadrupedTimedStep stepFinished : stepsFinished)
               currentFootstepPositions.put(stepFinished.getRobotQuadrant(), stepFinished.getGoalPosition());

            // remove steps in progress
            List<QuadrupedTimedStep> currentSteps = TimeIntervalTools.getIntervalsContainingTime(time, steps);
            for (QuadrupedTimedStep currentStep : currentSteps)
               currentFootstepPositions.put(currentStep.getRobotQuadrant(), null);

            footPositionScene.add(currentFootstepPositions);

            time += frameDt;
         }

         previewSlider.setBlockIncrement(1.0 / footPositionScene.size());
      }

      @Override
      public void start()
      {
         playbackCounter = 0;
         super.start();
         active.set(true);
         playbackModeActive.set(true);
      }

      @Override
      public void stop()
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            previewFootstepPositions.get(robotQuadrant).setToNaN();
         super.stop();
         active.set(false);
      }

      @Override
      public void handle(long now)
      {
         if (playbackModeActive.get())
         {
            if (footPositionScene.size() == 0)
            {
               LogTools.info("No footstep plan preview in memory.");
               playbackModeActive.set(false);
               stop();
               return;
            }

            if (playbackCounter >= footPositionScene.size())
            {
               playbackCounter = 0;
            }

            setToFrame(playbackCounter);
            playbackCounter += playbackSpeed;
            double alpha = ((double) playbackCounter) / (footPositionScene.size() - 1);
            previewSlider.setValue(MathTools.clamp(alpha, 0.0, 1.0));
         }
      }

      void requestSpecificPercentageInPreview(double alpha)
      {
         if (playbackModeActive.get())
            return;

         alpha = MathTools.clamp(alpha, 0.0, 1.0);

         if (footPositionScene.size() == 0)
         {
            LogTools.info("No footstep plan in memory.");
            playbackModeActive.set(false);
            stop();
            return;
         }

         int frameIndex = (int) (alpha * (footPositionScene.size() - 1));
         setToFrame(frameIndex);
      }

      private void setToFrame(int frameIndex)
      {
         if (footPositionScene.size() == 0)
         {
            LogTools.info("No footstep plan in memory.");
            playbackModeActive.set(false);
            stop();
            return;
         }

         if (frameIndex >= footPositionScene.size())
         {
            LogTools.info("frameIndex out of bound.");
            stop();
            return;
         }

         QuadrantDependentList<Point3DReadOnly> footPositions = footPositionScene.get(frameIndex);

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (footPositions.get(robotQuadrant) == null)
               previewFootstepPositions.get(robotQuadrant).setToNaN();
            else
               previewFootstepPositions.get(robotQuadrant).set(footPositions.get(robotQuadrant));
         }
      }
   }

   public void stop()
   {
      footstepPlanPreviewPlaybackManager.stop();
      executorService.shutdown();
   }
}
