package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.*;
import javafx.animation.AnimationTimer;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.cell.PropertyValueFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
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
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class MainTabController
{
   private static final boolean verbose = false;
   private static final double safetyRadiusToDiscardSteps = 0.8;
   private final ObservableList<RejectionPercentageProperty> rejectionTableItems = FXCollections.observableArrayList();
   private static final DecimalFormat percentageFormat = new DecimalFormat("#0.00");

   // control
   @FXML
   private ComboBox<FootstepPlannerType> plannerType;
   @FXML
   private CheckBox acceptNewRegions;
   @FXML
   private CheckBox assumeFlatGround;
   @FXML
   private CheckBox ignorePartialFootholds;
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
   private TableView rejectionTable;
   @FXML
   private TextField rejectionPercentage;

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
   private ComboBox<RobotSide> initialSupportSide;

   @FXML
   private Spinner<Double> startYaw;
   @FXML
   private Spinner<Double> goalYaw;

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

      setStartFromRobot();
      int newRequestID = currentPlannerRequestId.get() + 1;
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, newRequestID);
      messager.submitMessage(ComputePathTopic, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         LogTools.info("Clicked abort planning...");

      messager.submitMessage(AbortPlanningTopic, true);
   }

   @FXML
   public void sendPlan()
   {
      FootstepDataListMessage footstepDataListMessage = footstepPlanReference.get();
      if (footstepDataListMessage == null)
         return;

      if (overrideTiming.isSelected())
      {
         Object<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
         for (int i = 0; i < footstepDataList.size(); i++)
         {
            FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
            footstepDataMessage.setSwingDuration(swingTimeSpinner.getValue());
            footstepDataMessage.setTransferDuration(transferTimeSpinner.getValue());
         }
      }

      if (overrideSwingHeight.isSelected())
      {
         Object<FootstepDataMessage> footstepDataList = footstepDataListMessage.getFootstepDataList();
         for (int i = 0; i < footstepDataList.size(); i++)
         {
            FootstepDataMessage footstepDataMessage = footstepDataList.get(i);
            footstepDataMessage.setSwingHeight(swingHeightSpinner.getValue());
         }
      }

      us.ihmc.idl.IDLSequence.Object<controller_msgs.msg.dds.FootstepDataMessage> footstepSequence = footstepDataListMessage.getFootstepDataList();
      for (int i = 1; i < footstepSequence.size(); i++)
      {
         Point3D previousLocation = footstepSequence.get(i - 1).getLocation();
         Point3D location = footstepSequence.get(i).getLocation();

         if (previousLocation.distance(location) >= safetyRadiusToDiscardSteps)
         {
            footstepSequence.remove(i);
            i--;
         }
      }

      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanToRobotTopic, footstepDataListMessage);
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
      humanoidReferenceFrames.updateFrames();
      RigidBodyTransform transformToWorld = humanoidReferenceFrames.getMidFeetZUpFrame().getTransformToWorldFrame();
      ConvexPolygon2D convexPolygon = new ConvexPolygon2D();
      convexPolygon.addVertex(10.0, 10.0);
      convexPolygon.addVertex(-10.0, 10.0);
      convexPolygon.addVertex(-10.0, -10.0);
      convexPolygon.addVertex(10.0, -10.0);
      convexPolygon.update();
      PlanarRegion groundPlane = new PlanarRegion(transformToWorld, convexPolygon);
      return new PlanarRegionsList(groundPlane);
   }

   private JavaFXMessager messager;

   private AtomicReference<Integer> currentPlannerRequestId;
   private AnimationTimer robotPoseHandler;
   private WalkingPreviewPlaybackManager walkingPreviewPlaybackManager;
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private AtomicReference<FootstepDataListMessage> footstepPlanReference;
   private final ArrayList<List<Point3D>> contactPointHolder = new ArrayList<>();
   private SideDependentList<ArrayList<Point3D>> defaultContactPoints = new SideDependentList<>();

   private final Point3DProperty startPositionProperty = new Point3DProperty(this, "startPositionProperty", new Point3D());
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());

   private final YawProperty startRotationProperty = new YawProperty(this, "startRotationProperty", 0.0);
   private final YawProperty goalRotationProperty = new YawProperty(this, "goalRotationProperty", 0.0);

   private final AtomicInteger walkingPreviewRequestId = new AtomicInteger(0);

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

      currentPlannerRequestId = messager.createInput(FootstepPlannerMessagerAPI.PlannerRequestIdTopic, -1);
      footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanResponseTopic, null);
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

      swingTimeSpinner.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 3.5, 1.2, 0.1));
      transferTimeSpinner.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 3.5, 0.8, 0.1));
      swingHeightSpinner.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.0, 0.05, 0.01));

      overrideTiming.setSelected(true);
      overrideSwingHeight.setSelected(true);

      overrideTiming.selectedProperty().addListener(s ->
                                                    {
                                                       swingTimeSpinner.disableProperty().set(!overrideTiming.isSelected());
                                                       transferTimeSpinner.disableProperty().set(!overrideTiming.isSelected());
                                                    });

      overrideSwingHeight.selectedProperty().addListener(s -> swingHeightSpinner.disableProperty().set(!overrideSwingHeight.isSelected()));

      ObservableList<us.ihmc.footstepPlanning.FootstepPlannerType> plannerTypeOptions = FXCollections.observableArrayList(FootstepPlannerType.values);
      plannerType.setItems(plannerTypeOptions);
      plannerType.setValue(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR);

      timeout.setValueFactory(createTimeoutValueFactory());
      horizonLength.setValueFactory(createHorizonValueFactory());

      initialSupportSide.setItems(FXCollections.observableArrayList(RobotSide.values));
      initialSupportSide.setValue(RobotSide.LEFT);

      messager.bindTopic(IgnorePartialFootholdsTopic, ignorePartialFootholds.selectedProperty());
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

      messager.bindBidirectional(FootstepPlannerMessagerAPI.InitialSupportSideTopic, initialSupportSide.valueProperty(), true);

      startPositionProperty.bindBidirectionalX(startXPosition.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYPosition.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZPosition.getValueFactory().valueProperty());
      messager.bindBidirectional(StartPositionTopic, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYPosition.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZPosition.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionTopic, goalPositionProperty, false);

      messager.bindBidirectional(GoalPositionTopic, goalPositionProperty, false);

      startRotationProperty.bindBidirectionalYaw(startYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(StartOrientationTopic, startRotationProperty, false);

      goalRotationProperty.bindBidirectionalYaw(goalYaw.getValueFactory().valueProperty());
      messager.bindBidirectional(GoalOrientationTopic, goalRotationProperty, false);

      messager.registerTopicListener(GlobalResetTopic, reset -> clearStartGoalTextFields());

      walkingPreviewPlaybackManager = new WalkingPreviewPlaybackManager(messager);
      previewSlider.valueProperty().addListener((ChangeListener<Number>) (observable, oldValue, newValue) -> walkingPreviewPlaybackManager.requestSpecificPercentageInPreview(newValue.doubleValue()));

      setupRejectionTable();
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
      if (humanoidReferenceFrames == null)
         return;

      humanoidReferenceFrames.updateFrames();
      FramePose3D startPose = new FramePose3D(humanoidReferenceFrames.getSoleFrame(initialSupportSide.getValue()));
      startPose.changeFrame(ReferenceFrame.getWorldFrame());
      startPositionProperty.set(new Point3D(startPose.getPosition()));
      startRotationProperty.set(new Quaternion(startPose.getYaw(), 0.0, 0.0));
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

   public void setContactPointParameters(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ArrayList<Point3D> contactPoints = new ArrayList<>(contactPointParameters.getControllerFootGroundContactPoints().get(robotSide).stream()
                                                                                  .map(p -> new Point3D(p.getX(), p.getY(), 0.0)).collect(Collectors.toList()));
         defaultContactPoints.put(robotSide, contactPoints);
      }
   }

   public void setDefaultSwingHeight(double swingHeight)
   {
      swingHeightSpinner.getValueFactory().setValue(swingHeight);
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

   private SpinnerValueFactory.DoubleSpinnerValueFactory createStartGoalPositionValueFactory()
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createStartGoalOrientationValueFactory()
   {
      double min = -Math.PI;
      double max = Math.PI;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createTimeoutValueFactory()
   {
      double min = 0.0;
      double max = 500.0;
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

   private void setupRejectionTable()
   {
      TableColumn<String, RejectionPercentageProperty> column1 = new TableColumn<>("Reason");
      column1.setCellValueFactory(new PropertyValueFactory<>("reason"));

      TableColumn<String, RejectionPercentageProperty> column2 = new TableColumn<>("Percentage");
      column2.setCellValueFactory(new PropertyValueFactory<>("percentage"));

      column1.setSortable(false);
      column2.setSortable(false);

      rejectionTable.getColumns().add(column1);
      rejectionTable.getColumns().add(column2);

      rejectionTableItems.clear();
      for (BipedalFootstepPlannerNodeRejectionReason rejectionReason : BipedalFootstepPlannerNodeRejectionReason.values)
      {
         RejectionPercentageProperty rejectionPercentageProperty = new RejectionPercentageProperty(rejectionReason.toString(), percentageFormat.format(0.0));
         rejectionTableItems.add(rejectionPercentageProperty);
      }

      rejectionTable.setItems(rejectionTableItems);

      messager.registerTopicListener(PlannerStatisticsTopic, statisticsMessage ->
      {
         String percentageRejectionSteps = percentageFormat.format(100 * statisticsMessage.getFractionOfRejectedSteps());
         rejectionPercentage.setText(percentageRejectionSteps);

         rejectionTableItems.clear();
         for (BipedalFootstepPlannerNodeRejectionReason rejectionReason : BipedalFootstepPlannerNodeRejectionReason.values)
         {
            double percent = 100 * statisticsMessage.getRejectionFractions().get(rejectionReason.ordinal());
            RejectionPercentageProperty rejectionPercentageProperty = new RejectionPercentageProperty(rejectionReason.toString(), percentageFormat.format(percent));
            rejectionTableItems.add(rejectionPercentageProperty);
         }

         rejectionTableItems.sort(Collections.reverseOrder(Comparator.comparingDouble(p -> Double.parseDouble(p.percentage))));
         rejectionTable.setItems(rejectionTableItems);
      });
   }

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

   public class RejectionPercentageProperty
   {
      private String reason;
      private String percentage;

      public RejectionPercentageProperty(String reason, String percentage)
      {
         this.reason = reason;
         this.percentage = percentage;
      }

      public String getReason()
      {
         return reason;
      }

      public void setReason(String reason)
      {
         this.reason = reason;
      }

      public String getPercentage()
      {
         return percentage;
      }

      public void setPercentage(String percentage)
      {
         this.percentage = percentage;
      }
   }
}
