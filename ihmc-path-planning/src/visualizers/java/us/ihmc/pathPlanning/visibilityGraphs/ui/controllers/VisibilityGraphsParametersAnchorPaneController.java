package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphParametersKeys;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

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

   private final VisibilityGraphsParametersBasics planningParameters = new DefaultVisibilityGraphParameters();

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

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);
      javaFXStoredPropertyMap.put(maxInterRegionConnectionLengthSlider, VisibilityGraphParametersKeys.maxInterRegionConnectionLength);
      javaFXStoredPropertyMap.put(normalZThresholdForAccessibleRegionsSlider, VisibilityGraphParametersKeys.normalZThresholdForAccessibleRegions);
      javaFXStoredPropertyMap.put(extrusionDistanceSlider, VisibilityGraphParametersKeys.obstacleExtrusionDistance);
      javaFXStoredPropertyMap.put(extrusionDistanceIfNotTooHighToStepSlider, VisibilityGraphParametersKeys.obstacleExtrusionDistanceIfNotTooHighToStep);
      javaFXStoredPropertyMap.put(tooHighToStepDistanceSlider, VisibilityGraphParametersKeys.tooHighToStepDistance);
      javaFXStoredPropertyMap.put(clusterResolutionSlider, VisibilityGraphParametersKeys.clusterResolution);
      javaFXStoredPropertyMap.put(explorationDistanceFromStartGoalSlider, VisibilityGraphParametersKeys.explorationDistanceFromStartGoal);
      javaFXStoredPropertyMap.put(planarRegionMinAreaSlider, VisibilityGraphParametersKeys.planarRegionMinArea);
      javaFXStoredPropertyMap.putIntegerSlider(planarRegionMinSizeSlider, VisibilityGraphParametersKeys.planarRegionMinSize);
      javaFXStoredPropertyMap.put(regionOrthogonalAngleSlider, VisibilityGraphParametersKeys.regionOrthogonalAngle);

      // set messager updates to update all stored properties and select JavaFX properties
      messager.registerTopicListener(UIVisibilityGraphsTopics.VisibilityGraphsParameters, parameters ->
      {
         planningParameters.set(parameters);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      // set JavaFX user input to update stored properties and publish messager message
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(() -> publishParameters());
   }

   private void publishParameters()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.VisibilityGraphsParameters, planningParameters);
   }
}
