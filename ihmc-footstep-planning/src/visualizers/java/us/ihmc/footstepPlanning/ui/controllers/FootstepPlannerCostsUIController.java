package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlanningParameters;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class FootstepPlannerCostsUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty property = new FootstepPlannerParametersProperty();
   private FootstepPlanningParameters planningParameters;


   @FXML
   private CheckBox useQuadraticHeightCost;
   @FXML
   private CheckBox useQuadraticDistanceCost;

   @FXML
   private Spinner<Double> costPerStep;
   @FXML
   private Spinner<Double> aStarHeuristicsWeight;

   @FXML
   private Spinner<Double> yawWeight;
   @FXML
   private Spinner<Double> pitchWeight;
   @FXML
   private Spinner<Double> rollWeight;

   @FXML
   private Spinner<Double> forwardWeight;
   @FXML
   private Spinner<Double> lateralWeight;
   @FXML
   private Spinner<Double> stepUpWeight;
   @FXML
   private Spinner<Double> stepDownWeight;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlanningParameters parameters)
   {
      this.planningParameters = parameters;
      property.setPlannerParameters(parameters);
   }

   public void setupControls()
   {
      costPerStep.setValueFactory(createLowWeightValueFactory());
      aStarHeuristicsWeight.setValueFactory(createHighWeightValueFactory());

      yawWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.7, 0.0, 0.01));
      pitchWeight.setValueFactory(createLowWeightValueFactory());
      rollWeight.setValueFactory(createLowWeightValueFactory());

      forwardWeight.setValueFactory(createLowWeightValueFactory());
      lateralWeight.setValueFactory(createLowWeightValueFactory());
      stepUpWeight.setValueFactory(createLowWeightValueFactory());
      stepDownWeight.setValueFactory(createLowWeightValueFactory());
   }

   public void bindControls()
   {
      setupControls();

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParametersTopic, v -> planningParameters.set(v));


      property.bidirectionalBindUseQuadraticDistanceCost(useQuadraticDistanceCost.selectedProperty(), v -> publishParameters());
      property.bidirectionalBindUseQuadraticHeightCost(useQuadraticHeightCost.selectedProperty(), v -> publishParameters());

      property.bidirectionalBindCostPerStep(costPerStep.getValueFactory().valueProperty(), v -> publishParameters());
      property.bidirectionalBindAStarHeuristicsWeight(aStarHeuristicsWeight.getValueFactory().valueProperty(), v -> publishParameters());

      property.bidirectionalBindYawWeight(yawWeight.getValueFactory().valueProperty(), v -> publishParameters());
      property.bidirectionalBindPitchWeight(pitchWeight.getValueFactory().valueProperty(), v -> publishParameters());
      property.bidirectionalBindRollWeight(rollWeight.getValueFactory().valueProperty(), v -> publishParameters());

      property.bidirectionalBindForwardWeight(forwardWeight.getValueFactory().valueProperty(), v -> publishParameters());
      property.bidirectionalBindLateralWeight(lateralWeight.getValueFactory().valueProperty(), v -> publishParameters());
      property.bidirectionalBindStepUpWeight(stepUpWeight.getValueFactory().valueProperty(), v -> publishParameters());
      property.bidirectionalBindStepDownWeight(stepDownWeight.getValueFactory().valueProperty(), v -> publishParameters());
   }

   private void publishParameters()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, planningParameters);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createLowWeightValueFactory()
   {
      double min = 0.0;
      double max = 10.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createHighWeightValueFactory()
   {
      double min = 0.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }
}
