package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.NormalEstimationParametersProperty;

public class NormalEstimationAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;

   @FXML
   private Slider searchRadiusSlider;

   @FXML
   private Slider maxDistanceFromPlaneSlider;

   @FXML
   private Slider minConsensusRatioSlider;

   @FXML
   private Slider maxAverageDeviationRatioSlider;

   @FXML
   private Slider numberOfIterationsSlider;

   private final NormalEstimationParametersProperty normalEstimationParametersProperty = new NormalEstimationParametersProperty(this, "normalEstimationParameters");

   public NormalEstimationAnchorPaneController()
   {
   }

   private void setupControls()
   {
      searchRadiusSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxDistanceFromPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.NormalEstimationEnable, enableButton.selectedProperty());

      normalEstimationParametersProperty.bindBidirectionalSearchRadius(searchRadiusSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMinConsensusRatio(minConsensusRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxAverageDeviationRatio(maxAverageDeviationRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalNumberOfIterations(numberOfIterationsSlider.valueProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.NormalEstimationParameters, normalEstimationParametersProperty);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveMainUpdaterConfiguration);
   }

   @FXML
   public void resetNormals()
   {
      uiMessager.broadcastMessage(REAModuleAPI.NormalEstimationClear, true);
   }
}
