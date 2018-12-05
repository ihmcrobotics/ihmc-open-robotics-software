package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.SettableVisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.VisibilityGraphsParametersProperty;

public class VisibilityGraphsParametersAnchorPaneController
{
   @FXML
   private Slider maxInterRegionConnectionLengthSlider;
   @FXML
   private Slider normalZThresholdForAccessibleRegionsSlider;
   @FXML
   private Slider regionOrthogonalAngleSlider;
   @FXML
   private Slider extrusionDistanceSlider;
   @FXML
   private Slider extrusionDistanceIfNotTooHighToStepSlider;
   @FXML
   private Slider tooHighToStepDistanceSlider;
   @FXML
   private Slider clusterResolutionSlider;
   @FXML
   private Slider explorationDistanceFromStartGoalSlider;
   @FXML
   private Slider planarRegionMinAreaSlider;
   @FXML
   private Slider planarRegionMinSizeSlider;

   private final VisibilityGraphsParametersProperty property = new VisibilityGraphsParametersProperty(this, "visibilityGraphsParameters");
   private JavaFXMessager messager;

   public VisibilityGraphsParametersAnchorPaneController()
   {
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      maxInterRegionConnectionLengthSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      regionOrthogonalAngleSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());

      property.bidirectionalBindMaxInterRegionConnectionLength(maxInterRegionConnectionLengthSlider.valueProperty());
      property.bidirectionalBindNormalZThresholdForAccessibleRegions(normalZThresholdForAccessibleRegionsSlider.valueProperty());
      property.bidirectionalBindRegionOrthogonalAngle(regionOrthogonalAngleSlider.valueProperty());
      property.bidirectionalBindExtrusionDistance(extrusionDistanceSlider.valueProperty());
      property.bidirectionalBindExtrusionDistanceIfNotTooHighToStep(extrusionDistanceIfNotTooHighToStepSlider.valueProperty());
      property.bidirectionalBindTooHighToStepDistance(tooHighToStepDistanceSlider.valueProperty());
      property.bidirectionalBindClusterResolution(clusterResolutionSlider.valueProperty());
      property.bidirectionalBindExplorationDistanceFromStartGoal(explorationDistanceFromStartGoalSlider.valueProperty());
      property.bidirectionalBindPlanarRegionMinArea(planarRegionMinAreaSlider.valueProperty());
      property.bidirectionalBindPlanarRegionMinSize(planarRegionMinSizeSlider.valueProperty());

      property.set(new SettableVisibilityGraphsParameters(new DefaultVisibilityGraphParameters())); // Make sure the sliders are to the default values

      messager.bindBidirectional(UIVisibilityGraphsTopics.VisibilityGraphsParameters, property, createConverter(), true);
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
