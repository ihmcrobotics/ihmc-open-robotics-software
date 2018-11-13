package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty(this, "footstepPlannerParametersProperty");

   @FXML
   private ToggleButton returnBestEffortPlan;
   @FXML
   private ToggleButton performHeuristicSearchPolicies;
   @FXML
   private Spinner<Double> plannerTimeout;
   @FXML
   private Spinner<Double> horizonLength;

   @FXML
   private Spinner<Double> maxStepLength;
   @FXML
   private Spinner<Double> maxStepWidth;
   @FXML
   private Spinner<Double> minStepWidth;

   @FXML
   private Spinner<Double> minStepLength;
   @FXML
   private Spinner<Double> maxStepZ;
   @FXML
   private Spinner<Double> minSurfaceIncline;

   @FXML
   private Spinner<Double> maxStepYaw;
   @FXML
   private Spinner<Double> minStepYaw;
   @FXML
   private Spinner<Double> minFootholdPercent;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlannerParameters parameters)
   {
      parametersProperty.setPlannerParameters(parameters);
   }

   public void setupControls()
   {
      plannerTimeout.setValueFactory(createTimeoutValueFactory());
      horizonLength.setValueFactory(createHorizonValueFactory());
      maxStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.7, 0.0, 0.05));
      maxStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.02));
      minStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.01));

      minStepLength.setValueFactory(new DoubleSpinnerValueFactory(-0.6, 0.0, 0.0, 0.05));
      maxStepZ.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.02));
      minSurfaceIncline.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.1));


      maxStepYaw.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.1));
      minStepYaw.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.1));
      minFootholdPercent.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
   }

   public void bindControls()
   {
      setupControls();

      parametersProperty.bidirectionalBindReturnBestEffortPlan(returnBestEffortPlan.selectedProperty());
      parametersProperty.bidirectionalBindPerformHeuristicSearchPolicies(performHeuristicSearchPolicies.selectedProperty());

      parametersProperty.bidirectionalBindMaxStepReach(maxStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaxStepWidth(maxStepWidth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinStepWidth(minStepWidth.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMinStepLength(minStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaxStepZ(maxStepZ.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinSurfaceIncline(minSurfaceIncline.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaxStepYaw(maxStepYaw.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinStepYaw(minStepYaw.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinFootholdPercent(minFootholdPercent.getValueFactory().valueProperty());


      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerTimeoutTopic, plannerTimeout.getValueFactory().valueProperty(), doubleToDoubleConverter, true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerHorizonLengthTopic, horizonLength.getValueFactory().valueProperty(), doubleToDoubleConverter, true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerParametersTopic, parametersProperty, createConverter(), true);

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

   private PropertyToMessageTypeConverter<FootstepPlannerParameters, SettableFootstepPlannerParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<FootstepPlannerParameters, SettableFootstepPlannerParameters>()
      {
         @Override
         public FootstepPlannerParameters convert(SettableFootstepPlannerParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettableFootstepPlannerParameters interpret(FootstepPlannerParameters messageContent)
         {
            return new SettableFootstepPlannerParameters(messageContent);
         }
      };
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
      double max = 1000.0;
      double amountToStepBy = 0.25;
      return new DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }



}
