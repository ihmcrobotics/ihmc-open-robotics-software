package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.YawProperty;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

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

      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartPositionEditModeEnabled, startPositionToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabled, goalPositionToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabled, startRotationToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled, goalRotationToggleButton.selectedProperty(), false);

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabled, startPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabled, startPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled, startPositionToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartPositionEditModeEnabled, goalPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabled, goalPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled, goalPositionToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartPositionEditModeEnabled, startRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabled, startRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalOrientationEditModeEnabled, startRotationToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartPositionEditModeEnabled, goalRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.GoalPositionEditModeEnabled, goalRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerMessagerAPI.StartOrientationEditModeEnabled, goalRotationToggleButton.disableProperty());

      startPositionProperty.bindBidirectionalX(startXSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(StartPosition, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZSpinner.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.GoalPosition, goalPositionProperty, false);

      messager.bindBidirectional(GoalPosition, goalPositionProperty, false);

      startRotationProperty.bindBidirectionalYaw(startYawSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(StartOrientation, startRotationProperty, false);

      goalRotationProperty.bindBidirectionalYaw(goalYawSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(GoalOrientation, goalRotationProperty, false);

      messager.registerTopicListener(GlobalReset, reset -> clearStartGoalTextFields());
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
