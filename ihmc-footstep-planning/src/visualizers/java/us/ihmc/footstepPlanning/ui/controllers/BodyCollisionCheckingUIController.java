package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.graphSearch.parameters.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class BodyCollisionCheckingUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty(this, "bodyCollisionParametersProperty");

   @FXML
   private CheckBox enableBodyCollisionChecking;

   @FXML
   private Spinner<Double> bodyDepth;
   @FXML
   private Spinner<Double> bodyWidth;
   @FXML
   private Spinner<Double> bodyHeight;

   @FXML
   private Spinner<Double> bodyBoxBaseX;
   @FXML
   private Spinner<Double> bodyBoxBaseY;
   @FXML
   private Spinner<Double> bodyBoxBaseZ;

   @FXML
   private Spinner<Double> maximum2dDistanceFromBoundingBoxToPenalize;
   @FXML
   private Spinner<Double> boundingBoxCost;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setupControls()
   {
      bodyWidth.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      bodyDepth.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
      bodyHeight.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 2.0, 0.0, 0.1));

      bodyBoxBaseX.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-1.0, 1.0, 0.0, 0.05));
      bodyBoxBaseY.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-1.0, 1.0, 0.0, 0.05));
      bodyBoxBaseZ.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.1));

      maximum2dDistanceFromBoundingBoxToPenalize.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
      boundingBoxCost.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 10000.0, 0.0, 10.0));
   }

   public void bindControls()
   {
      setupControls();

      parametersProperty.bidirectionalBindCheckBodyBoxCollisions(enableBodyCollisionChecking.selectedProperty());

      parametersProperty.bidirectionalBindBodyBoxDepth(bodyDepth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindBodyBoxHeight(bodyHeight.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindBodyBoxWidth(bodyWidth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseX(bodyBoxBaseX.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseY(bodyBoxBaseY.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindBodyBoxBaseZ(bodyBoxBaseZ.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindBoundingBoxCost(boundingBoxCost.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximum2dDistanceFromBoundingBoxToPenalize(maximum2dDistanceFromBoundingBoxToPenalize.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerParametersTopic, parametersProperty, createConverter(), true);
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
