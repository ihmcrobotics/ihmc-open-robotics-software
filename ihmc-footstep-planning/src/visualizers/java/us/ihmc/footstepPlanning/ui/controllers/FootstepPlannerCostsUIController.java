package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

public class FootstepPlannerCostsUIController
{
   private JavaFXMessager messager;
   private FootstepPlannerParametersBasics planningParameters;


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

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      this.planningParameters = parameters;
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

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters.getStoredPropertySet());
      javaFXStoredPropertyMap.put(useQuadraticDistanceCost, FootstepPlannerParameterKeys.useQuadraticDistanceCost);
      javaFXStoredPropertyMap.put(useQuadraticHeightCost, FootstepPlannerParameterKeys.useQuadraticHeightCost);
      javaFXStoredPropertyMap.put(costPerStep, FootstepPlannerParameterKeys.costPerStep);
      javaFXStoredPropertyMap.put(aStarHeuristicsWeight, FootstepPlannerParameterKeys.aStarHeuristicsWeight);
      javaFXStoredPropertyMap.put(yawWeight, FootstepPlannerParameterKeys.yawWeight);
      javaFXStoredPropertyMap.put(pitchWeight, FootstepPlannerParameterKeys.pitchWeight);
      javaFXStoredPropertyMap.put(rollWeight, FootstepPlannerParameterKeys.rollWeight);
      javaFXStoredPropertyMap.put(forwardWeight, FootstepPlannerParameterKeys.forwardWeight);
      javaFXStoredPropertyMap.put(lateralWeight, FootstepPlannerParameterKeys.lateralWeight);
      javaFXStoredPropertyMap.put(stepUpWeight, FootstepPlannerParameterKeys.stepUpWeight);
      javaFXStoredPropertyMap.put(stepDownWeight, FootstepPlannerParameterKeys.stepDownWeight);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParametersTopic, v ->
      {
         planningParameters.set(v);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });


      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(() -> publishParameters());
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
