package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.YawProperty;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;

public class StartGoalTabController
{

   @FXML
   private ToggleButton startPositionToggleButton;
   @FXML
   private ToggleButton goalPositionToggleButton;

   @FXML
   private ToggleButton startRotationToggleButton;
   @FXML
   private ToggleButton goalRotationToggleButton;

   @FXML
   private Spinner<Double> startXSpinner;
   @FXML
   private Spinner<Double> startYSpinner;
   @FXML
   private Spinner<Double> startZSpinner;
   @FXML
   private Spinner<Double> goalXSpinner;
   @FXML
   private Spinner<Double> goalYSpinner;
   @FXML
   private Spinner<Double> goalZSpinner;

   @FXML
   private Spinner<Double> startYawSpinner;
   @FXML
   private Spinner<Double> goalYawSpinner;


   private JavaFXMessager messager;
   private final Point3DProperty startPositionProperty = new Point3DProperty(this, "startPositionProperty", new Point3D());
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());

   private final YawProperty startRotationProperty = new YawProperty(this, "startRotationProperty", 0.0);
   private final YawProperty goalRotationProperty = new YawProperty(this, "goalRotationProperty", 0.0);

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   private void setupControls()
   {
      startXSpinner.setValueFactory(createStartGoalPositionValueFactory());
      startYSpinner.setValueFactory(createStartGoalPositionValueFactory());
      startZSpinner.setValueFactory(createStartGoalPositionValueFactory());

      goalXSpinner.setValueFactory(createStartGoalPositionValueFactory());
      goalYSpinner.setValueFactory(createStartGoalPositionValueFactory());
      goalZSpinner.setValueFactory(createStartGoalPositionValueFactory());

      startYawSpinner.setValueFactory(createStartGoalOrientationValueFactory());
      goalYawSpinner.setValueFactory(createStartGoalOrientationValueFactory());

   }

   public void bindControls()
   {
      setupControls();

      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartPositionEditModeEnabledTopic, startPositionToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, goalPositionToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabledTopic, startRotationToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabledTopic, goalRotationToggleButton.selectedProperty(), false);

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, startPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabledTopic, startPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabledTopic, startPositionToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartPositionEditModeEnabledTopic, goalPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabledTopic, goalPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabledTopic, goalPositionToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartPositionEditModeEnabledTopic, startRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, startRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabledTopic, startRotationToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartPositionEditModeEnabledTopic, goalRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabledTopic, goalRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabledTopic, goalRotationToggleButton.disableProperty());

      startPositionProperty.bindBidirectionalX(startXSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartPositionTopic, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZSpinner.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionTopic, goalPositionProperty, false);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionTopic, goalPositionProperty, false);

      startRotationProperty.bindBidirectionalYaw(startYawSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartOrientationTopic, startRotationProperty, false);

      goalRotationProperty.bindBidirectionalYaw(goalYawSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalOrientationTopic, goalRotationProperty, false);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.GlobalResetTopic, reset -> clearStartGoalTextFields());
   }

   private void clearStartGoalTextFields()
   {
      startXSpinner.valueFactoryProperty().getValue().setValue(0.0);
      startYSpinner.valueFactoryProperty().getValue().setValue(0.0);
      startZSpinner.valueFactoryProperty().getValue().setValue(0.0);

      goalXSpinner.valueFactoryProperty().getValue().setValue(0.0);
      goalYSpinner.valueFactoryProperty().getValue().setValue(0.0);
      goalZSpinner.valueFactoryProperty().getValue().setValue(0.0);

      startYawSpinner.valueFactoryProperty().getValue().setValue(0.0);
      goalYawSpinner.valueFactoryProperty().getValue().setValue(0.0);
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
}
