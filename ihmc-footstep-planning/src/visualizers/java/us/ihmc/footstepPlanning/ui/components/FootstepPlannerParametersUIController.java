package us.ihmc.footstepPlanning.ui.components;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class FootstepPlannerParametersUIController
{
   private static final boolean verbose = false;
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty property = new FootstepPlannerParametersProperty(this, "footstepPlannerParametersProperty");

   @FXML
   private Slider maxStepReach;

   @FXML
   private Slider maxStepYaw;

   @FXML
   private Slider minStepWidth;

   @FXML
   private Slider minStepLength;

   @FXML
   private Slider minStepYaw;

   @FXML
   private Slider maxStepZ;

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



   public void bindControls()
   {
//      property.bidirectionalBindIdealFootstepWidth();
//      property.bidirectionalBindIdealFootstepLength();
      property.bidirectionalBindMaxStepReach(maxStepReach.valueProperty());
      property.bidirectionalBindMaxStepYaw(maxStepYaw.valueProperty());
      property.bidirectionalBindMinStepWidth(minStepWidth.valueProperty());
      property.bidirectionalBindMinStepLength(minStepLength.valueProperty());
      property.bidirectionalBindMinStepYaw(minStepYaw.valueProperty());
      property.bidirectionalBindMaxStepZ(maxStepZ.valueProperty());
      property.bidirectionalBindMinFootholdPercent(minFootholdPercent.valueProperty());
      property.bidirectionalBindMinSurfaceIncline(minSurfaceIncline.valueProperty());
      property.bidirectionalBindMaxStepWidth(maxStepWidth.valueProperty());


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




}
