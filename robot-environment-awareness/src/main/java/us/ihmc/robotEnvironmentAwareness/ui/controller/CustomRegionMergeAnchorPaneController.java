package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
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

   private final PlanarRegionSegmentationParametersProperty customRegionMergingParametersProperty = new PlanarRegionSegmentationParametersProperty(this,
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

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.CustomRegionsMergingEnable, enableMergeButton.selectedProperty());

      customRegionMergingParametersProperty.bindBidirectionalSearchRadius(maxDistanceInPlaneSlider.valueProperty());
      customRegionMergingParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      customRegionMergingParametersProperty.bindBidirectionalMaxAngleFromPlane(maxAngleFromPlaneSlider.valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.CustomRegionsMergingParameters, customRegionMergingParametersProperty);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveRegionUpdaterConfiguration);
   }

   @FXML
   public void clear()
   {
      uiMessager.submitMessageToModule(REAModuleAPI.CustomRegionsClear, true);
   }
}
