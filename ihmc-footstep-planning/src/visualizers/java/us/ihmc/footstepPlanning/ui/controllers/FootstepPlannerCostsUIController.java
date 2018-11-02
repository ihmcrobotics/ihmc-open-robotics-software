package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerCostParametersProperty;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.javaFXToolkit.messager.TopicListener;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerCostsUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerCostParametersProperty property = new FootstepPlannerCostParametersProperty(this, "footstepPlannerCostParametersProperty");

   @FXML
   private ToggleButton useQuadraticHeightCost;
   @FXML
   private ToggleButton useQuadraticDistanceCost;

   @FXML
   private Spinner<Double> costPerStep;
   @FXML
   private Spinner<Double> heuristicsWeight;

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

   private SpinnerValueFactory<Double> heuristicsWeightValueFactory;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlannerParameters parameters)
   {
      property.setPlannerParameters(parameters.getCostParameters());
   }

   public void setupControls()
   {
      costPerStep.setValueFactory(createLowWeightValueFactory());
      heuristicsWeight.setValueFactory(createHighWeightValueFactory());

      yawWeight.setValueFactory(createLowWeightValueFactory());
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

      heuristicsWeightValueFactory = heuristicsWeight.getValueFactory();

      AtomicReference<FootstepPlannerType> plannerType = messager.createInput(FootstepPlannerMessagerAPI.PlannerTypeTopic);
      AtomicReference<FootstepPlannerCostParameters> plannerParameters = messager.createInput(FootstepPlannerMessagerAPI.PlannerCostParametersTopic);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerTypeTopic, createPlannerTypeChangeListener(plannerType, plannerParameters));

      property.bidirectionalBindUseQuadraticDistanceCost(useQuadraticDistanceCost.selectedProperty());
      property.bidirectionalBindUseQuadraticHeightCost(useQuadraticHeightCost.selectedProperty());

      property.bidirectionalBindCostPerStep(costPerStep.getValueFactory().valueProperty());
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerTypeTopic, createPlannerTypeChangeListener(plannerType, plannerParameters));
      property.bidirectionalBindHeuristicsWeight(plannerType, heuristicsWeightValueFactory.valueProperty());

      property.bidirectionalBindYawWeight(yawWeight.getValueFactory().valueProperty());
      property.bidirectionalBindPitchWeight(pitchWeight.getValueFactory().valueProperty());
      property.bidirectionalBindRollWeight(rollWeight.getValueFactory().valueProperty());

      property.bidirectionalBindForwardWeight(forwardWeight.getValueFactory().valueProperty());
      property.bidirectionalBindLateralWeight(lateralWeight.getValueFactory().valueProperty());
      property.bidirectionalBindStepUpWeight(stepUpWeight.getValueFactory().valueProperty());
      property.bidirectionalBindStepDownWeight(stepDownWeight.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerCostParametersTopic, property, createConverter(), true);
   }

   private PropertyToMessageTypeConverter<FootstepPlannerCostParameters, SettableFootstepPlannerCostParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<FootstepPlannerCostParameters, SettableFootstepPlannerCostParameters>()
      {
         @Override
         public FootstepPlannerCostParameters convert(SettableFootstepPlannerCostParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettableFootstepPlannerCostParameters interpret(FootstepPlannerCostParameters messageContent)
         {
            return new SettableFootstepPlannerCostParameters(messageContent);
         }
      };
   }

   private TopicListener<FootstepPlannerType> createPlannerTypeChangeListener(AtomicReference<FootstepPlannerType> plannerType, AtomicReference<FootstepPlannerCostParameters> plannerParameters)
   {
      return new TopicListener<FootstepPlannerType>()
      {
         @Override
         public void receivedMessageForTopic(FootstepPlannerType messageContent)
         {
            FootstepPlannerType footstepPlannerType = plannerType.get();
            if (footstepPlannerType == null)
               return;

            double weight = 0.0;
            switch (footstepPlannerType)
            {
            case A_STAR:
               weight = plannerParameters.get().getAStarHeuristicsWeight().getValue();
               break;
            case VIS_GRAPH_WITH_A_STAR:
               weight = plannerParameters.get().getVisGraphWithAStarHeuristicsWeight().getValue();
               break;
            case PLANAR_REGION_BIPEDAL:
               weight = plannerParameters.get().getDepthFirstHeuristicsWeight().getValue();
               break;
            case SIMPLE_BODY_PATH:
               weight = plannerParameters.get().getBodyPathBasedHeuristicsWeight().getValue();
               break;
            default:
               weight = 0.0;
               break;
            }

            heuristicsWeightValueFactory.setValue(weight);
         }
      };
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
