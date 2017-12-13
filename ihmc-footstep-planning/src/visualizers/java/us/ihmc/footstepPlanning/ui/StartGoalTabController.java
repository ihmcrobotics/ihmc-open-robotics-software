package us.ihmc.footstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.ui.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics;

public class StartGoalTabController
{
   @FXML
   private ToggleButton placeStartToggleButton;
   @FXML
   private ToggleButton placeGoalToggleButton;
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

   private SimpleUIMessager messager;
   private final Point3DProperty startPositionProperty = new Point3DProperty(this, "startPositionProperty", new Point3D());
   private final Point3DProperty goalPositionProperty = new Point3DProperty(this, "goalPositionProperty", new Point3D());

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   private void setupControls()
   {
      startXSpinner.setValueFactory(createStartGoalValueFactory());
      startYSpinner.setValueFactory(createStartGoalValueFactory());
      startZSpinner.setValueFactory(createStartGoalValueFactory());

      goalXSpinner.setValueFactory(createStartGoalValueFactory());
      goalYSpinner.setValueFactory(createStartGoalValueFactory());
      goalZSpinner.setValueFactory(createStartGoalValueFactory());
   }

   public void bindControls()
   {
      setupControls();

      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.StartEditModeEnabledTopic, placeStartToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.StartEditModeEnabledTopic, placeGoalToggleButton.disableProperty(), false);

      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.GoalEditModeEnabledTopic, placeGoalToggleButton.selectedProperty(), false);
      messager.bindBidirectional(FootstepPlannerUserInterfaceAPI.GoalEditModeEnabledTopic, placeStartToggleButton.disableProperty(), false);

      startPositionProperty.bindBidirectionalX(startXSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(UIVisibilityGraphsTopics.StartPosition, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(UIVisibilityGraphsTopics.GoalPosition, goalPositionProperty, false);

      messager.registerTopicListener(UIVisibilityGraphsTopics.GlobalReset, reset -> clearStartGoalTextFields());
   }

   private void clearStartGoalTextFields()
   {
      startXSpinner.valueFactoryProperty().getValue().setValue(0.0);
      startYSpinner.valueFactoryProperty().getValue().setValue(0.0);
      startZSpinner.valueFactoryProperty().getValue().setValue(0.0);

      goalXSpinner.valueFactoryProperty().getValue().setValue(0.0);
      goalYSpinner.valueFactoryProperty().getValue().setValue(0.0);
      goalZSpinner.valueFactoryProperty().getValue().setValue(0.0);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createStartGoalValueFactory()
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

}
