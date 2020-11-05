package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.WalkingControllerPreviewInputMessage;
import controller_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import javafx.animation.AnimationTimer;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.FootstepPlanHeading;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadRequestType;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Float;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.YawProperty;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class MainTabController
{
   private static final boolean verbose = false;

   // control
   @FXML
   private CheckBox acceptNewRegions;
   @FXML
   private CheckBox assumeFlatGround;
   @FXML
   private CheckBox planBodyPath;
   @FXML
   private CheckBox performAStarSearch;
   @FXML
   private CheckBox ignorePartialFootholds;
   @FXML
   private Spinner<Double> timeout;
   @FXML
   private Spinner<Double> horizonLength;
   @FXML
   private ComboBox<SwingPlannerType> swingPlannerType;

   @FXML
   private ComboBox<DataSetName> dataSetSelector;
   @FXML
   private TextField logGenerationStatus;
   @FXML
   private TextField logLoadStatus;

   // goal placement
   @FXML
   private Button placeGoal;
   @FXML
   private CheckBox bindStartToRobot;
   @FXML
   private CheckBox snapGoalSteps;
   @FXML
   private CheckBox abortIfGoalStepSnapFails;

   @FXML
   private Button computePath;
   @FXML
   private Button postProcess;
   @FXML
   private Button abortPlanning;

   @FXML
   private Spinner<Double> goalXPosition;
   @FXML
   private Spinner<Double> goalYPosition;
   @FXML
   private Spinner<Double> goalZPosition;
   @FXML
   private ComboBox<RobotSide> initialSupportSide;
   @FXML
   private Spinner<Double> pathHeading;

   @FXML
   private Spinner<Double> goalYaw;

   @FXML
   private Spinner<Double> distanceProximity;
   @FXML
   private Spinner<Double> yawProximity;

   @FXML
   private CheckBox overrideTiming;
   @FXML
   private Spinner<Double> swingTimeSpinner;
   @FXML
   private Spinner<Double> transferTimeSpinner;
   @FXML
   private CheckBox overrideSwingHeight;
   @FXML
   private Spinner<Double> swingHeightSpinner;

   @FXML
   private Slider previewSlider;

   @FXML
   public void computePath()
   {
      if (verbose)
         LogTools.info("Clicked compute path...");

      if (bindStartToRobot.isSelected())
        setStartFromRobot();
      int newRequestID = currentPlannerRequestId.get() + 1;
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, newRequestID);
      messager.submitMessage(ComputePath, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         LogTools.info("Clicked abort planning...");

      messager.submitMessage(HaltPlanning, true);
   }

   @FXML
   public void postProcess()
   {
      if (verbose)
         LogTools.info("Clicked post process...");
      messager.submitMessage(PostProcessPlan, true);
   }

   @FXML
   public void sendPlan()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.SendPlan, true);
   }

   @FXML
   public void clearFlat()
   {
      acceptNewRegions.setSelected(false);
      assumeFlatGround.setSelected(true);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, buildFlatGround());
   }

   private PlanarRegionsList buildFlatGround()
   {
      humanoidReferenceFrames.updateFrames();
      return PlanarRegionsList.flatGround(20.0, humanoidReferenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame());
   }

   private JavaFXMessager messager;

   private AtomicReference<Integer> currentPlannerRequestId;
   private WalkingPreviewPlaybackManager walkingPreviewPlaybackManager;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private AtomicReference<FootstepDataListMessage> footstepPlanReference;
   private final ArrayList<List<Point3D>> contactPointHolder = new ArrayList<>();
   private SideDependentList<List<Point3D>> defaultContactPoints = new SideDependentList<>();
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());
   private final YawProperty goalRotationProperty = new YawProperty(this, "goalRotationProperty", 0.0);

   private final AtomicInteger walkingPreviewRequestId = new AtomicInteger(0);

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentPlannerRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestId, -1);
      footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponse, null);
   }

   private void setupControls()
   {
      goalXPosition.setValueFactory(createGoalPositionValueFactory());
      goalYPosition.setValueFactory(createGoalPositionValueFactory());
      goalZPosition.setValueFactory(createGoalPositionValueFactory());
      goalYaw.setValueFactory(createGoalOrientationValueFactory());

      distanceProximity.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-100.0, 100.0, 0.0, 0.1));
      yawProximity.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, Math.PI, 0.0, 0.1));

      swingTimeSpinner.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(- Double.MAX_VALUE, Double.MAX_VALUE, 1.2, 0.1));
      transferTimeSpinner.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(- Double.MAX_VALUE, Double.MAX_VALUE, 0.8, 0.1));
      swingHeightSpinner.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.0, 0.05, 0.01));

      overrideTiming.setSelected(false);
      overrideSwingHeight.setSelected(false);

      overrideTiming.selectedProperty().addListener(s ->
                                                    {
                                                       swingTimeSpinner.disableProperty().set(!overrideTiming.isSelected());
                                                       transferTimeSpinner.disableProperty().set(!overrideTiming.isSelected());
                                                    });

      overrideSwingHeight.selectedProperty().addListener(s -> swingHeightSpinner.disableProperty().set(!overrideSwingHeight.isSelected()));

      timeout.setValueFactory(createTimeoutValueFactory());
      horizonLength.setValueFactory(createHorizonValueFactory());
      pathHeading.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(- Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));

      initialSupportSide.setItems(FXCollections.observableArrayList(RobotSide.values));
      initialSupportSide.setValue(RobotSide.LEFT);

      swingPlannerType.setItems(FXCollections.observableArrayList(SwingPlannerType.values()));
      swingPlannerType.setValue(SwingPlannerType.NONE);

      messager.bindBidirectional(IgnorePartialFootholds, ignorePartialFootholds.selectedProperty(), true);
      messager.bindBidirectional(OverrideStepTimings, overrideTiming.selectedProperty(), true);
      messager.bindBidirectional(ManualSwingTime, swingTimeSpinner.valueFactoryProperty().getValue().valueProperty(), true);
      messager.bindBidirectional(ManualTransferTime, transferTimeSpinner.valueFactoryProperty().getValue().valueProperty(), true);

      messager.bindBidirectional(OverrideSwingHeight, overrideSwingHeight.selectedProperty(), true);
      messager.bindBidirectional(ManualSwingHeight, swingHeightSpinner.valueFactoryProperty().getValue().valueProperty(), true);

      messager.bindBidirectional(SnapGoalSteps, snapGoalSteps.selectedProperty(), true);
      messager.bindBidirectional(AbortIfGoalStepSnapFails, abortIfGoalStepSnapFails.selectedProperty(), true);

      dataSetSelector.setItems(FXCollections.observableArrayList(DataSetName.values()));
      dataSetSelector.valueProperty().addListener(((observable, oldValue, newValue) -> messager.submitMessage(DataSetSelected, dataSetSelector.getValue())));
   }

   public void bindControls()
   {
      setupControls();

      messager.bindBidirectional(FootstepPlannerMessagerAPI.AcceptNewPlanarRegions, acceptNewRegions.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerTimeout, timeout.getValueFactory().valueProperty(), doubleToDoubleConverter, true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerHorizonLength, horizonLength.getValueFactory().valueProperty(), doubleToDoubleConverter,
                                 true);

      // set goal
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabled, placeGoal.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabled, computePath.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.EditModeEnabled, abortPlanning.disableProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.AssumeFlatGround, assumeFlatGround.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.InitialSupportSide, initialSupportSide.valueProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.RequestedFootstepPlanHeading, pathHeading.getValueFactory().valueProperty(), false);

      goalPositionProperty.bindBidirectionalX(goalXPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZPosition.getValueFactory().valueProperty());

      messager.bindBidirectional(GoalMidFootPosition, goalPositionProperty, false);

      messager.bindBidirectional(GoalDistanceProximity, distanceProximity.getValueFactory().valueProperty(), true);
      messager.bindBidirectional(GoalYawProximity, yawProximity.getValueFactory().valueProperty(), true);

      goalRotationProperty.bindBidirectionalYaw(goalYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(GoalMidFootOrientation, goalRotationProperty, false);

      messager.bindBidirectional(PlanBodyPath, planBodyPath.selectedProperty(), true);
      messager.bindBidirectional(PerformAStarSearch, performAStarSearch.selectedProperty(), true);
      messager.bindBidirectional(RequestedSwingPlannerType, swingPlannerType.valueProperty(), true);

      walkingPreviewPlaybackManager = new WalkingPreviewPlaybackManager(messager);
      previewSlider.valueProperty().addListener((ChangeListener<Number>) (observable, oldValue, newValue) -> walkingPreviewPlaybackManager.requestSpecificPercentageInPreview(newValue.doubleValue()));

      messager.registerTopicListener(GenerateLogStatus, logGenerationStatus::setText);
      messager.registerTopicListener(LoadLogStatus, logLoadStatus::setText);
   }

   @FXML
   public void placeGoal()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabled, true);
      messager.submitMessage(FootstepPlannerMessagerAPI.EditModeEnabled, true);
   }

   @FXML
   public void generateLog()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.RequestGenerateLog, true);
   }

   @FXML
   public void loadLog()
   {
      messager.submitMessage(RequestLoadLog, LoadRequestType.FILE_CHOOSER);
   }

   @FXML
   public void loadLatestLog()
   {
      messager.submitMessage(RequestLoadLog, LoadRequestType.LATEST);
   }

   @FXML
   public void loadPreviousLog()
   {
      messager.submitMessage(RequestLoadLog, LoadRequestType.PREVIOUS);
   }

   @FXML
   public void loadNextLog()
   {
      messager.submitMessage(RequestLoadLog, LoadRequestType.NEXT);
   }

   private void setStartFromRobot()
   {
      if (humanoidReferenceFrames == null)
         return;

      humanoidReferenceFrames.updateFrames();
      FramePose3D leftFootPose = new FramePose3D(humanoidReferenceFrames.getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(humanoidReferenceFrames.getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      messager.submitMessage(LeftFootPose, leftFootPose);
      messager.submitMessage(RightFootPose, rightFootPose);
   }

   @FXML
   private void requestWalkingPreview()
   {
      WalkingControllerPreviewInputMessage requestMessage = new WalkingControllerPreviewInputMessage();
      requestMessage.setSequenceId(walkingPreviewRequestId.incrementAndGet());

      FootstepDataListMessage footstepDataListMessage = footstepPlanReference.get();
      if (footstepDataListMessage == null)
         return;

      requestMessage.getFootsteps().set(footstepDataListMessage);
      requestMessage.getFootsteps().setOffsetFootstepsWithExecutionError(false);
      messager.submitMessage(FootstepPlannerMessagerAPI.RequestWalkingPreview, requestMessage);
   }

   @FXML
   private void playWalkingPreview()
   {
      walkingPreviewPlaybackManager.start();
   }

   @FXML
   private void pauseWalkingPreview()
   {
      walkingPreviewPlaybackManager.playbackModeActive.set(false);
   }

   @FXML
   private void stopWalkingPreview()
   {
      walkingPreviewPlaybackManager.stop();
   }

   public void setFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);
   }

   public void setPreviewModel(FullHumanoidRobotModel previewRobotModel)
   {
      this.walkingPreviewPlaybackManager.setRobotModel(previewRobotModel);
   }

   public void setDefaultTiming(double swingTime, double transferTime)
   {
      swingTimeSpinner.getValueFactory().setValue(swingTime);
      transferTimeSpinner.getValueFactory().setValue(transferTime);
   }

   public void setContactPointParameters(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.defaultContactPoints.put(robotSide,
                                       defaultContactPoints.get(robotSide)
                                                           .stream()
                                                           .map(p -> new Point3D(p.getX(), p.getY(), 0.0))
                                                           .collect(Collectors.toList()));
      }
   }

   public void setDefaultSwingHeight(double swingHeight)
   {
      swingHeightSpinner.getValueFactory().setValue(swingHeight);
   }

   private void clearGoalTextFields()
   {
      goalXPosition.valueFactoryProperty().getValue().setValue(0.0);
      goalYPosition.valueFactoryProperty().getValue().setValue(0.0);
      goalZPosition.valueFactoryProperty().getValue().setValue(0.0);
      goalYaw.valueFactoryProperty().getValue().setValue(0.0);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createGoalPositionValueFactory()
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createGoalOrientationValueFactory()
   {
      double min = -Math.PI;
      double max = Math.PI;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createTimeoutValueFactory()
   {
      double min = 0.0;
      double max = Double.MAX_VALUE;
      double amountToStepBy = 5;
      return new DoubleSpinnerValueFactory(min, max, 15.0, amountToStepBy);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createHorizonValueFactory()
   {
      double min = 0.0;
      double max = 100.0;
      double defaultValue = 10.0;
      double amountToStepBy = 0.5;
      return new DoubleSpinnerValueFactory(min, max, defaultValue, amountToStepBy);
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

   private class WalkingPreviewPlaybackManager extends AnimationTimer
   {
      final AtomicReference<WalkingControllerPreviewOutputMessage> walkingPreviewOutput;

      // frames per call to handle()
      final int playbackSpeed = 1;

      int playbackCounter = 0;
      FullHumanoidRobotModel previewRobotModel = null;
      OneDoFJointBasics[] previewModelOneDoFJoints = null;

      // whether to show ghost robot
      final AtomicBoolean active = new AtomicBoolean(false);

      // whether to animate ghost robot
      final AtomicBoolean playbackModeActive = new AtomicBoolean(false);

      WalkingPreviewPlaybackManager(Messager messager)
      {
         walkingPreviewOutput = messager.createInput(FootstepPlannerMessagerAPI.WalkingPreviewOutput);
         messager.registerTopicListener(FootstepPlannerMessagerAPI.WalkingPreviewOutput, output -> start());
      }

      void setRobotModel(FullHumanoidRobotModel previewRobotModel)
      {
         this.previewRobotModel = previewRobotModel;
         previewModelOneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(previewRobotModel);
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
         previewRobotModel.getRootJoint().setJointPosition(new Vector3D(Double.NaN, Double.NaN, Double.NaN));
         super.stop();
         active.set(false);
      }

      @Override
      public void handle(long now)
      {
         if (playbackModeActive.get())
         {
            WalkingControllerPreviewOutputMessage walkingControllerPreviewOutputMessage = walkingPreviewOutput.get();

            if (walkingControllerPreviewOutputMessage == null)
            {
               LogTools.info("No preview in memory.");
               playbackModeActive.set(false);
               stop();
               return;
            }

            if (playbackCounter >= walkingControllerPreviewOutputMessage.getRobotConfigurations().size())
            {
               playbackCounter = 0;
            }

            setToFrame(playbackCounter);
            playbackCounter += playbackSpeed;
            double alpha = ((double) playbackCounter) / (walkingPreviewOutput.get().getRobotConfigurations().size() - 1);
            previewSlider.setValue(MathTools.clamp(alpha, 0.0, 1.0));
         }
      }

      void requestSpecificPercentageInPreview(double alpha)
      {
         if (playbackModeActive.get())
            return;

         alpha = MathTools.clamp(alpha, 0.0, 1.0);
         WalkingControllerPreviewOutputMessage walkingControllerPreviewOutputMessage = walkingPreviewOutput.get();

         if (walkingControllerPreviewOutputMessage == null)
         {
            LogTools.info("No preview in memory.");
            playbackModeActive.set(false);
            stop();
            return;
         }

         int frameIndex = (int) (alpha * (walkingControllerPreviewOutputMessage.getRobotConfigurations().size() - 1));
         setToFrame(frameIndex);
      }

      private void setToFrame(int frameIndex)
      {
         WalkingControllerPreviewOutputMessage walkingControllerPreviewOutputMessage = walkingPreviewOutput.get();

         if (walkingControllerPreviewOutputMessage == null)
         {
            LogTools.info("No preview in memory.");
            playbackModeActive.set(false);
            stop();
            return;
         }

         Object<KinematicsToolboxOutputStatus> robotConfigurations = walkingControllerPreviewOutputMessage.getRobotConfigurations();

         if (frameIndex >= robotConfigurations.size())
         {
            LogTools.info("frameIndex out of bound.");
            stop();
            return;
         }

         KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus = robotConfigurations.get(frameIndex);

         Float jointAngles = kinematicsToolboxOutputStatus.getDesiredJointAngles();

         if (jointAngles.size() != previewModelOneDoFJoints.length)
         {
            System.err.println("Received " + jointAngles.size() + " from walking controller preview toolbox, expected " + previewModelOneDoFJoints.length);
            walkingPreviewOutput.set(null);
            return;
         }

         for (int i = 0; i < jointAngles.size(); i++)
         {
            previewModelOneDoFJoints[i].setQ(jointAngles.get(i));
         }

         previewRobotModel.getRootJoint().setJointPosition(kinematicsToolboxOutputStatus.getDesiredRootTranslation());
         previewRobotModel.getRootJoint().setJointOrientation(kinematicsToolboxOutputStatus.getDesiredRootOrientation());
      }
   }
}
