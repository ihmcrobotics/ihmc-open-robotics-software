package us.ihmc.footstepPlanning.ui.components;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.javaFXToolkit.messager.TopicListener;

import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerCostsUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty property = new FootstepPlannerParametersProperty(this, "footstepPlannerCostParametersProperty");

   @FXML
   private Slider yawWeight;

   @FXML
   private Slider costPerStep;

   @FXML
   private Slider heuristicsWeight;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      AtomicReference<FootstepPlannerType> plannerType = messager.createInput(FootstepPlannerMessagerAPI.PlannerTypeTopic);
      AtomicReference<FootstepPlannerParameters> plannerParameters = messager.createInput(FootstepPlannerMessagerAPI.PlannerParametersTopic);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerTypeTopic, createPlannerTypeChangeListener(plannerType, plannerParameters));

      property.bidirectionalBindYawWeight(yawWeight.valueProperty());
      property.bidirectionalBindCostPerStep(costPerStep.valueProperty());

      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerTypeTopic, createPlannerTypeChangeListener(plannerType, plannerParameters));
      property.bidirectionalBindHeuristicsWeight(plannerType, heuristicsWeight.valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerParametersTopic, property, createConverter(), true);
   }

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

   private TopicListener<FootstepPlannerType> createPlannerTypeChangeListener(AtomicReference<FootstepPlannerType> plannerType, AtomicReference<FootstepPlannerParameters> plannerParameters)
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
               weight = plannerParameters.get().getCostParameters().getAStarHeuristicsWeight().getValue();
               break;
            case VIS_GRAPH_WITH_A_STAR:
               weight = plannerParameters.get().getCostParameters().getVisGraphWithAStarHeuristicsWeight().getValue();
               break;
            case PLANAR_REGION_BIPEDAL:
               weight = plannerParameters.get().getCostParameters().getDepthFirstHeuristicsWeight().getValue();
               break;
            case SIMPLE_BODY_PATH:
               weight = plannerParameters.get().getCostParameters().getBodyPathBasedHeuristicsWeight().getValue();
               break;
            default:
               weight = 0.0;
               break;
            }

            heuristicsWeight.setValue(weight);
         }
      };
   }
}
