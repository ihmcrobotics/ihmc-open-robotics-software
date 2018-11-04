package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.SettableVisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.VisibilityGraphsParametersProperty;

public class VisibilityGraphsParametersUIController
{
   private JavaFXMessager messager;
   private final VisibilityGraphsParametersProperty parametersProperty = new VisibilityGraphsParametersProperty(this, "visibilityGraphsParametersProperty");

   @FXML
   private Slider maxInterRegionConnectionLength;
   @FXML
   private Slider normalZThresholdForAccessibleRegions;
   @FXML
   private Slider extrusionDistance;
   @FXML
   private Slider extrusionDistanceIfNotTooHighToStep;
   @FXML
   private Slider tooHighToStepDistance;
   @FXML
   private Slider clusterResolution;
   @FXML
   private Slider explorationDistanceFromStartGoal;
   @FXML
   private Slider planarRegionMinArea;

   @FXML
   private Spinner<Integer> planarRegionMinSize;
   @FXML
   private Slider regionOrthogonalAngle;
   @FXML
   private Slider searchHostRegionEpsilon;



   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setVisbilityGraphsParameters(VisibilityGraphsParameters parameters)
   {
      parametersProperty.setPlannerParameters(parameters);
   }

   private void setupControls()
   {
      planarRegionMinSize.setValueFactory(createPlanarRegionMinSizeValueFactory());
   }

   public void bindControls()
   {
      setupControls();

      parametersProperty.bidirectionalBindMaxInterRegionConnectionLength(maxInterRegionConnectionLength.valueProperty());
      parametersProperty.bidirectionalBindNormalZThresholdForAccessibleRegions(normalZThresholdForAccessibleRegions.valueProperty());
      parametersProperty.bidirectionalBindExtrusionDistance(extrusionDistance.valueProperty());
      parametersProperty.bidirectionalBindExtrusionDistanceIfNotTooHighToStep(extrusionDistanceIfNotTooHighToStep.valueProperty());
      parametersProperty.bidirectionalBindTooHighToStepDistance(tooHighToStepDistance.valueProperty());
      parametersProperty.bidirectionalBindClusterResolution(clusterResolution.valueProperty());
      parametersProperty.bidirectionalBindExplorationDistanceFromStartGoal(explorationDistanceFromStartGoal.valueProperty());
      parametersProperty.bidirectionalBindPlanarRegionMinArea(planarRegionMinArea.valueProperty());
      parametersProperty.bidirectionalBindPlanarRegionMinSize(planarRegionMinSize.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindRegionOrthogonalAngle(regionOrthogonalAngle.valueProperty());
      parametersProperty.bidirectionalBindSearchHostRegionEpsilon(searchHostRegionEpsilon.valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.VisibilityGraphsParametersTopic, parametersProperty, createConverter(), true);
   }

   private SpinnerValueFactory.IntegerSpinnerValueFactory createPlanarRegionMinSizeValueFactory()
   {
      int min = 0;
      int max = 100;
      int amountToStepBy = 1;
      return new SpinnerValueFactory.IntegerSpinnerValueFactory(min, max, 0, amountToStepBy);
   }

   private PropertyToMessageTypeConverter<VisibilityGraphsParameters, SettableVisibilityGraphsParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<VisibilityGraphsParameters, SettableVisibilityGraphsParameters>()
      {
         @Override
         public VisibilityGraphsParameters convert(SettableVisibilityGraphsParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettableVisibilityGraphsParameters interpret(VisibilityGraphsParameters messageContent)
         {
            return new SettableVisibilityGraphsParameters(messageContent);
         }
      };
   }
}
