package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.VisibilityGraphsParametersProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.VisibilityGraphsParametersProperty.SettableVisibilityGraphsParameters;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

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
   private SimpleUIMessager messager;

   public VisibilityGraphsParametersAnchorPaneController()
   {
   }

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      maxInterRegionConnectionLengthSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      regionOrthogonalAngleSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());

      property.binBidirectionalMaxInterRegionConnectionLength(maxInterRegionConnectionLengthSlider.valueProperty());
      property.binBidirectionalNormalZThresholdForAccessibleRegions(normalZThresholdForAccessibleRegionsSlider.valueProperty());
      property.binBidirectionalRegionOrthogonalAngle(regionOrthogonalAngleSlider.valueProperty());
      property.binBidirectionalExtrusionDistance(extrusionDistanceSlider.valueProperty());
      property.binBidirectionalExtrusionDistanceIfNotTooHighToStep(extrusionDistanceIfNotTooHighToStepSlider.valueProperty());
      property.binBidirectionalTooHighToStepDistance(tooHighToStepDistanceSlider.valueProperty());
      property.binBidirectionalClusterResolution(clusterResolutionSlider.valueProperty());
      property.binBidirectionalExplorationDistanceFromStartGoal(explorationDistanceFromStartGoalSlider.valueProperty());
      property.binBidirectionalPlanarRegionMinArea(planarRegionMinAreaSlider.valueProperty());
      property.binBidirectionalPlanarRegionMinSize(planarRegionMinSizeSlider.valueProperty());

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
