package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
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
   private Spinner<Double> plannerTimeout;
   @FXML
   private Spinner<Double> horizonLength;

   @FXML
   private Slider maxStepLength;
   @FXML
   private Slider minStepWidth;
   @FXML
   private Slider maxStepYaw;

   @FXML
   private Slider minStepLength;
   @FXML
   private Slider maxStepZ;
   @FXML
   private Slider minStepYaw;

   @FXML
   private Slider minFootholdPercent;
   @FXML
   private Slider minSurfaceIncline;
   @FXML
   private Slider maxStepWidth;


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
   }

   public void bindControls()
   {
      setupControls();

//      parametersProperty.bidirectionalBindIdealFootstepWidth();
//      parametersProperty.bidirectionalBindIdealFootstepLength();
      parametersProperty.bidirectionalBindMaxStepReach(maxStepLength.valueProperty());
      parametersProperty.bidirectionalBindMaxStepYaw(maxStepYaw.valueProperty());
      parametersProperty.bidirectionalBindMinStepWidth(minStepWidth.valueProperty());
      parametersProperty.bidirectionalBindMinStepLength(minStepLength.valueProperty());
      parametersProperty.bidirectionalBindMinStepYaw(minStepYaw.valueProperty());
      parametersProperty.bidirectionalBindMaxStepZ(maxStepZ.valueProperty());
      parametersProperty.bidirectionalBindMinFootholdPercent(minFootholdPercent.valueProperty());
      parametersProperty.bidirectionalBindMinSurfaceIncline(minSurfaceIncline.valueProperty());
      parametersProperty.bidirectionalBindMaxStepWidth(maxStepWidth.valueProperty());


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
      double max = 100.0;
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
