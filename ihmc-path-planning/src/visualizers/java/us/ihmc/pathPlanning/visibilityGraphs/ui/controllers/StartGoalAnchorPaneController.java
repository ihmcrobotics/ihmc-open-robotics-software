package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalEditModeEnabled;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowGoalPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowStartPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartEditModeEnabled;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;

public class StartGoalAnchorPaneController
{
   @FXML
   private ToggleButton showStartToggleButton;
   @FXML
   private ToggleButton showGoalToggleButton;
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

      messager.bindBidirectional(ShowStartPosition, showStartToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowGoalPosition, showGoalToggleButton.selectedProperty(), false);
      
      messager.bindBidirectional(StartEditModeEnabled, placeStartToggleButton.selectedProperty(), false);
      messager.bindBidirectional(StartEditModeEnabled, placeGoalToggleButton.disableProperty(), false);

      messager.bindBidirectional(GoalEditModeEnabled, placeGoalToggleButton.selectedProperty(), false);
      messager.bindBidirectional(GoalEditModeEnabled, placeStartToggleButton.disableProperty(), false);


      startPositionProperty.bindBidirectionalX(startXSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalY(startYSpinner.getValueFactory().valueProperty());
      startPositionProperty.bindBidirectionalZ(startZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(UIVisibilityGraphsTopics.StartPosition, startPositionProperty, false);

      goalPositionProperty.bindBidirectionalX(goalXSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalY(goalYSpinner.getValueFactory().valueProperty());
      goalPositionProperty.bindBidirectionalZ(goalZSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(UIVisibilityGraphsTopics.GoalPosition, goalPositionProperty, false);
   }

   private DoubleSpinnerValueFactory createStartGoalValueFactory()
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }
}
