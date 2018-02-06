package us.ihmc.footstepPlanning.ui;

import javafx.beans.property.SimpleObjectProperty;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

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

   private SimpleUIMessager messager;
   private final Point3DProperty startPositionProperty = new Point3DProperty(this, "startPositionProperty", new Point3D());
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());

   private final SimpleObjectProperty<Double> startRotationProperty = new SimpleObjectProperty<>(this, "startRotationProperty", 0.0);
   private final SimpleObjectProperty<Double> goalRotationProperty = new SimpleObjectProperty<>(this, "goalRotationProperty", 0.0);

   public void attachMessager(SimpleUIMessager messager)
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

      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.StartPositionEditModeEnabledTopic, startPositionToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.GoalPositionEditModeEnabledTopic, goalPositionToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.StartOrientationEditModeEnabledTopic, startRotationToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.GoalOrientationEditModeEnabledTopic, goalRotationToggleButton.selectedProperty(), false);

      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.GoalPositionEditModeEnabledTopic, startPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.StartOrientationEditModeEnabledTopic, startPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.GoalOrientationEditModeEnabledTopic, startPositionToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.StartPositionEditModeEnabledTopic, goalPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.StartOrientationEditModeEnabledTopic, goalPositionToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.GoalOrientationEditModeEnabledTopic, goalPositionToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.StartPositionEditModeEnabledTopic, startRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.GoalPositionEditModeEnabledTopic, startRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.GoalOrientationEditModeEnabledTopic, startRotationToggleButton.disableProperty());

      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.StartPositionEditModeEnabledTopic, goalRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.GoalPositionEditModeEnabledTopic, goalRotationToggleButton.disableProperty());
      messager.bindPropertyToTopic(FootstepPlannerUserInterfaceAPI.StartOrientationEditModeEnabledTopic, goalRotationToggleButton.disableProperty());

      startPositionProperty.bindBidirectionalX(startXSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(StartPositionTopic, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZSpinner.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.GoalPositionTopic, goalPositionProperty, false);

      messager.bindBidirectional(GoalPositionTopic, goalPositionProperty, false);

      startRotationProperty.bindBidirectional(startYawSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(StartOrientationTopic, startRotationProperty, false);

      goalRotationProperty.bindBidirectional(goalYawSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(GoalOrientationTopic, goalRotationProperty, false);

      messager.registerTopicListener(GlobalResetTopic, reset -> clearStartGoalTextFields());
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
