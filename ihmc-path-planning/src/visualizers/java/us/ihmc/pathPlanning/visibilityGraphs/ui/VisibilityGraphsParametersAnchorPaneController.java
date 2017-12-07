package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VisibilityGraphsParametersProperty.SettableVisibilityGraphsParameters;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class VisibilityGraphsParametersAnchorPaneController
{
   @FXML
   private Slider numberOfForcedConnectionsSlider;
   @FXML
   private Slider minConnectionDistanceForRegionsSlider;
   @FXML
   private Slider normalZThresholdForAccessibleRegionsSlider;
   @FXML
   private Slider normalZThresholdForPolygonObstaclesSlider;
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
      minConnectionDistanceForRegionsSlider.setLabelFormatter(StringConverterTools.metersToRoundedMillimeters());

      property.binBidirectionalNumberOfForcedConnections(numberOfForcedConnectionsSlider.valueProperty());
      property.binBidirectionalMinimumConnectionDistanceForRegions(minConnectionDistanceForRegionsSlider.valueProperty());
      property.binBidirectionalNormalZThresholdForAccessibleRegions(normalZThresholdForAccessibleRegionsSlider.valueProperty());
      property.binBidirectionalNormalZThresholdForPolygonObstacles(normalZThresholdForPolygonObstaclesSlider.valueProperty());
      property.binBidirectionalExtrusionDistance(extrusionDistanceSlider.valueProperty());
      property.binBidirectionalExtrusionDistanceIfNotTooHighToStep(extrusionDistanceIfNotTooHighToStepSlider.valueProperty());
      property.binBidirectionalTooHighToStepDistance(tooHighToStepDistanceSlider.valueProperty());
      property.binBidirectionalClusterResolution(clusterResolutionSlider.valueProperty());
      property.binBidirectionalExplorationDistanceFromStartGoal(explorationDistanceFromStartGoalSlider.valueProperty());
      property.binBidirectionalPlanarRegionMinArea(planarRegionMinAreaSlider.valueProperty());
      property.binBidirectionalPlanarRegionMinSize(planarRegionMinSizeSlider.valueProperty());

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
