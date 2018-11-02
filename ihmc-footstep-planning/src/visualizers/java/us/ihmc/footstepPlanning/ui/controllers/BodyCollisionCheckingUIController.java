package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.BodyCollisionPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.BodyCollisionPlannerParametersProperty;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.ui.components.SettableBodyCollisionPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class BodyCollisionCheckingUIController
{
   private JavaFXMessager messager;
   private final BodyCollisionPlannerParametersProperty parametersProperty = new BodyCollisionPlannerParametersProperty(this, "bodyCollisionParametersProperty");

   @FXML
   private ToggleButton enableBodyCollisionChecking;

   @FXML
   private Slider bodyWidth;
   @FXML
   private Slider bodyDepth;
   @FXML
   private Slider bodyHeight;

   @FXML
   private Slider bodyBoxBaseX;
   @FXML
   private Slider bodyBoxBaseY;
   @FXML
   private Slider bodyBoxBaseZ;


   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   
   public void bindControls()
   {
      parametersProperty.bidirectionalBindCheckBodyBoxCollisions(enableBodyCollisionChecking.selectedProperty());

      parametersProperty.bidirectionalBindBodyBoxDepth(bodyDepth.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxHeight(bodyHeight.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxWidth(bodyWidth.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseX(bodyBoxBaseX.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseY(bodyBoxBaseY.valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseZ(bodyBoxBaseZ.valueProperty());

//      messager.bindBidirectional(FootstepPlannerMessagerAPI.BodyCollisionParametersTopic, parametersProperty, createConverter(), true);
   }


   private PropertyToMessageTypeConverter<BodyCollisionPlannerParameters, SettableBodyCollisionPlannerParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<BodyCollisionPlannerParameters, SettableBodyCollisionPlannerParameters>()
      {
         @Override
         public BodyCollisionPlannerParameters convert(SettableBodyCollisionPlannerParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettableBodyCollisionPlannerParameters interpret(BodyCollisionPlannerParameters messageContent)
         {
            return new SettableBodyCollisionPlannerParameters(messageContent);
         }
      };
   }
}
