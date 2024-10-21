package us.ihmc.footstepPlanning.ui.controllers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.AbortIfGoalStepSnapFails;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ApproveStep;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.BindStartToRobot;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePath;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.DataSetSelected;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GenerateLogStatus;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalDistanceProximity;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalMidFootOrientation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalMidFootPosition;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalYawProximity;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.HaltPlanning;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.HeightMapDataSetSelected;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.IgnorePartialFootholds;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.LeftFootPose;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.LoadLogStatus;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ManualSwingHeight;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ManualSwingTime;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ManualTransferTime;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.OverrideStepTimings;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.OverrideSwingHeight;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PerformAStarSearch;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanBodyPath;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanSingleStep;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ReconnectRos1Node;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ReplanStep;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ReplanSwing;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RequestLoadLog;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RequestedSwingPlannerType;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ResendLastStep;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RightFootPose;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.SnapGoalSteps;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartHeightMapNavigation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StopHeightMapNavigation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.WriteHeightMapLog;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import controller_msgs.msg.dds.FootstepDataListMessage;
import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.TextField;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadRequestType;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.HeightMapDataSetName;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.YawProperty;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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
   private CheckBox planSingleStep;
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
   private ComboBox<HeightMapDataSetName> heightMapDataSetSelector;
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
   public void computeSwing()
   {
      messager.submitMessage(ReplanSwing, true);
   }

   @FXML
   public void abortPlanning()
   {
      if (verbose)
         LogTools.info("Clicked abort planning...");

      messager.submitMessage(HaltPlanning, true);
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
   private HumanoidReferenceFrames humanoidReferenceFrames;
   private AtomicReference<FootstepDataListMessage> footstepPlanReference;
   private final ArrayList<List<Point3D>> contactPointHolder = new ArrayList<>();
   private SideDependentList<List<Point3D>> defaultContactPoints = new SideDependentList<>();
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());
   private final YawProperty goalRotationProperty = new YawProperty(this, "goalRotationProperty", 0.0);

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

      messager.bindBidirectional(BindStartToRobot, bindStartToRobot.selectedProperty(), true);
      messager.bindBidirectional(PlanSingleStep, planSingleStep.selectedProperty(), true);
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

      heightMapDataSetSelector.setItems(FXCollections.observableArrayList(HeightMapDataSetName.values()));
      heightMapDataSetSelector.valueProperty().addListener(((observable, oldValue, newValue) -> messager.submitMessage(HeightMapDataSetSelected, heightMapDataSetSelector.getValue())));
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

      messager.addTopicListener(GenerateLogStatus, logGenerationStatus::setText);
      messager.addTopicListener(LoadLogStatus, logLoadStatus::setText);
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

   @FXML
   public void startHeightMapNavigation()
   {
      messager.submitMessage(StartHeightMapNavigation, true);
   }

   @FXML
   public void stopHeightMapNavigation()
   {
      messager.submitMessage(StopHeightMapNavigation, true);
   }

   @FXML
   public void approve()
   {
      messager.submitMessage(ApproveStep, true);
   }

   @FXML
   public void replan()
   {
      messager.submitMessage(ReplanStep, true);
   }

   @FXML
   public void writeHeightMapLog()
   {
      messager.submitMessage(WriteHeightMapLog, true);
   }

   @FXML
   public void resendLastStep()
   {
      messager.submitMessage(ResendLastStep, true);
   }

   @FXML
   public void reconnect()
   {
      messager.submitMessage(ReconnectRos1Node, true);
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

   public void setFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);
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
}
