package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.CustomRegionMergeParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.CustomRegionMergeParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PlanarRegionSegmentationParametersProperty;

public class CustomRegionMergeAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableMergeButton;
   @FXML
   private Slider maxDistanceInPlaneSlider;
   @FXML
   private Slider maxDistanceFromPlaneSlider;
   @FXML
   private Slider maxAngleFromPlaneSlider;

   private final CustomRegionMergeParametersProperty customRegionMergingParametersProperty = new CustomRegionMergeParametersProperty(this,
                                                                                                                                            "customRegionMergingParameters");

   public CustomRegionMergeAnchorPaneController()
   {
   }

   private void setupControls()
   {
      maxDistanceInPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxDistanceFromPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxAngleFromPlaneSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
   }

   private Topic<Boolean> customRegionsMergingEnableTopic = REAModuleAPI.CustomRegionsMergingEnable;
   private Topic<Boolean> customRegionsClearTopic = REAModuleAPI.CustomRegionsClear;
   private Topic<Boolean> saveRegionUpdaterConfigurationTopic = REAModuleAPI.SaveRegionUpdaterConfiguration;
   private Topic<CustomRegionMergeParameters> customRegionsMergingParametersTopic = REAModuleAPI.CustomRegionsMergingParameters;

   public void setCustomRegionsMergingEnableTopic(Topic<Boolean> customRegionsMergingEnableTopic)
   {
      this.customRegionsMergingEnableTopic = customRegionsMergingEnableTopic;
   }

   public void setCustomRegionsClearTopic(Topic<Boolean> customRegionsClearTopic)
   {
      this.customRegionsClearTopic = customRegionsClearTopic;
   }

   public void setSaveRegionUpdaterConfigurationTopic(Topic<Boolean> saveRegionUpdaterConfigurationTopic)
   {
      this.saveRegionUpdaterConfigurationTopic = saveRegionUpdaterConfigurationTopic;
   }

   public void setCustomRegionsMergingParametersTopic(Topic<CustomRegionMergeParameters> customRegionsMergingParametersTopic)
   {
      this.customRegionsMergingParametersTopic = customRegionsMergingParametersTopic;
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(customRegionsMergingEnableTopic, enableMergeButton.selectedProperty());

      customRegionMergingParametersProperty.bindBidirectionalSearchRadius(maxDistanceInPlaneSlider.valueProperty());
      customRegionMergingParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      customRegionMergingParametersProperty.bindBidirectionalMaxAngleFromPlane(maxAngleFromPlaneSlider.valueProperty());
      uiMessager.bindBidirectionalGlobal(customRegionsMergingParametersTopic, customRegionMergingParametersProperty);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(saveRegionUpdaterConfigurationTopic);
   }

   @FXML
   public void clear()
   {
      uiMessager.submitMessageToModule(customRegionsClearTopic, true);
   }
}
