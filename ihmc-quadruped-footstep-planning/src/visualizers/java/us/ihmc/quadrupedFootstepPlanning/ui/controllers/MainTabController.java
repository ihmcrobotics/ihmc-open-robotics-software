package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.WalkingControllerPreviewInputMessage;
import controller_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import javafx.animation.AnimationTimer;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.messager.Messager;
import us.ihmc.messager.TopicListener;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.YawProperty;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.ui.viewers.FootstepPlannerProcessViewer;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedRobotics.estimator.GroundPlaneEstimator;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalTools;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI.AbortPlanningTopic;
import static us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePathTopic;

public class MainTabController
{
   private static final boolean verbose = true;

   // control
   @FXML
   private ComboBox<FootstepPlannerType> plannerType;
   @FXML
   private CheckBox acceptNewRegions;
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
   public void computePath()
   {
      if (verbose)
         PrintTools.info(this, "Clicked compute path...");

      if (quadrupedReferenceFrames != null)
         setStartFromRobot();

      int newRequestID = currentPlannerRequestId.get() + 1;
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, newRequestID);
      messager.submitMessage(ComputePathTopic, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         PrintTools.info(this, "Clicked abort planning...");

      messager.submitMessage(AbortPlanningTopic, true);
   }

   @FXML
   public void sendPlan()
   {
      FootstepPlan footstepPlan = footstepPlanReference.get();
      if (footstepPlan == null)
         return;
      double swingTime = 1.2;
      double transferTime = 0.8;
      throw new RuntimeException("This feature has not yet been implemented.");
//      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, swingTime, transferTime,
//                                                                                                                    ExecutionMode.OVERRIDE);
//      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepDataListTopic, footstepDataListMessage);
   }

   @FXML
   public void clearFlat()
   {
      acceptNewRegions.setSelected(false);
      assumeFlatGround.setSelected(true);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionDataTopic, buildFlatGround());
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
   private AtomicReference<FootstepPlan> footstepPlanReference;

   private final Point3DProperty startPositionProperty = new Point3DProperty(this, "startPositionProperty", new Point3D());
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());

   private final YawProperty startRotationProperty = new YawProperty(this, "startRotationProperty", 0.0);
   private final YawProperty goalRotationProperty = new YawProperty(this, "goalRotationProperty", 0.0);

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

      currentPlannerRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, -1);
      footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanTopic, null);
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


      ObservableList<FootstepPlannerType> plannerTypeOptions = FXCollections.observableArrayList(FootstepPlannerType.values);
      plannerType.setItems(plannerTypeOptions);
      plannerType.setValue(FootstepPlannerType.A_STAR);

      timeout.setValueFactory(createTimeoutValueFactory());
      horizonLength.setValueFactory(createHorizonValueFactory());

      initialSupportQuadrant.setItems(FXCollections.observableArrayList(RobotQuadrant.values));
      initialSupportQuadrant.setValue(RobotQuadrant.FRONT_LEFT);
   }

   public void bindControls()
   {
      setupControls();

      // control
      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerTypeTopic, plannerType.valueProperty(), true);
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, new TextViewerListener<>(sentRequestId));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.ReceivedPlanIdTopic, new TextViewerListener<>(receivedRequestId));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerTimeTakenTopic, new TextViewerListener<>(timeTaken));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlanningResultTopic, new TextViewerListener<>(planningResult));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerStatusTopic, new TextViewerListener<>(plannerStatus));

      messager.bindBidirectional(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, acceptNewRegions.selectedProperty(), true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerTimeoutTopic, timeout.getValueFactory().valueProperty(), doubleToDoubleConverter, true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerHorizonLengthTopic, horizonLength.getValueFactory().valueProperty(), doubleToDoubleConverter,
                                 true);

      // set goal
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabledTopic, placeStart.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabledTopic, placeGoal.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabledTopic, computePath.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabledTopic, abortPlanning.disableProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.AssumeFlatGround, assumeFlatGround.selectedProperty(), false);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.InitialSupportQuadrantTopic, initialSupportQuadrant.valueProperty(), true);

      startPositionProperty.bindBidirectionalX(startXPosition.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYPosition.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZPosition.getValueFactory().valueProperty());
      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartPositionTopic, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZPosition.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionTopic, goalPositionProperty, false);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionTopic, goalPositionProperty, false);

      startRotationProperty.bindBidirectionalYaw(startYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartOrientationTopic, startRotationProperty, false);

      goalRotationProperty.bindBidirectionalYaw(goalYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalOrientationTopic, goalRotationProperty, false);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.GlobalResetTopic, reset -> clearStartGoalTextFields());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerPlaybackFractionTopic, previewSlider.valueProperty(), false);

      //      walkingPreviewPlaybackManager = new WalkingPreviewPlaybackManager(messager);
      footstepPlanPreviewPlaybackManager = new FootstepPlanPreviewPlaybackManager(messager);
      previewSlider.valueProperty()
                   .addListener((observable, oldValue, newValue) -> footstepPlanPreviewPlaybackManager.requestSpecificPercentageInPreview(newValue.doubleValue()));


   }

   @FXML
   public void placeStart()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPositionEditModeEnabledTopic, true);
      messager.submitMessage(FootstepPlannerMessagerAPI.EditModeEnabledTopic, true);
   }

   @FXML
   public void placeGoal()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, true);
      messager.submitMessage(FootstepPlannerMessagerAPI.EditModeEnabledTopic, true);
   }

   private void setStartFromRobot()
   {
      quadrupedReferenceFrames.updateFrames();
      FramePose3D startPose = new FramePose3D(quadrupedReferenceFrames.getSoleFrame(initialSupportQuadrant.getValue()));
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      startPositionProperty.set(new Point3D(startPose.getPosition()));
      startRotationProperty.set(new Quaternion(startPose.getYaw(), 0.0, 0.0));
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
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowFootstepPreviewTopic, true);
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
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowFootstepPreviewTopic, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowFootstepPlanTopic, true);
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
      private final AtomicReference<FootstepPlan> footstepPlanReference;
      private final AtomicReference<QuadrupedXGaitSettingsReadOnly> xGaitSettingsReference;

      private final double frameDt = 0.01;
      // frames per call to handle()
      final int playbackSpeed = 5;
      int playbackCounter = 0;

      // whether to show ghost robot
      final AtomicBoolean active = new AtomicBoolean(false);

      // whether to animate ghost robot
      final AtomicBoolean playbackModeActive = new AtomicBoolean(false);
      private final List<QuadrantDependentList<Point3DReadOnly>> footPositionScene = new ArrayList<>();

      QuadrantDependentList<Point3D> previewFootstepPositions;

      FootstepPlanPreviewPlaybackManager(Messager messager)
      {
         footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanTopic);
         xGaitSettingsReference = messager.createInput(FootstepPlannerMessagerAPI.XGaitSettingsTopic);

         messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanTopic, output -> calculateFrames());
      }

      void setPreviewFootstepPositions(QuadrantDependentList<Point3D> previewFootstepPositions)
      {
         this.previewFootstepPositions = previewFootstepPositions;
      }

      private void calculateFrames()
      {
         FootstepPlan footstepPlan = footstepPlanReference.getAndSet(null);
         List<QuadrupedTimedStep> steps = new ArrayList<>();
         for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
            steps.add(footstepPlan.getFootstep(i));
         TimeIntervalTools.sortByEndTime(steps);
         double endTime = steps.get(footstepPlan.getNumberOfSteps() - 1).getTimeInterval().getEndTime();
         TimeIntervalTools.sortByStartTime(steps);
         double startTime = steps.get(0).getTimeInterval().getStartTime() - 1.0;

         footPositionScene.clear();

         PoseReferenceFrame xGaitFrame = new PoseReferenceFrame("xGaitFrame", ReferenceFrame.getWorldFrame());
         xGaitFrame.setPoseAndUpdate(footstepPlan.getStartPose());

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


}
