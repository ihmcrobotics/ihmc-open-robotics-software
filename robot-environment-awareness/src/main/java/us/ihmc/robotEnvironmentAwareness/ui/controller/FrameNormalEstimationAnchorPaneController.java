package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.NormalEstimationParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceElementICPSLAMParametersProperty;

public class FrameNormalEstimationAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;

   @FXML
   private ToggleButton weightNumberOfHits;
   @FXML
   private ToggleButton enableLeastSquares;

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
   private final SurfaceElementICPSLAMParametersProperty slamParametersProperty = new SurfaceElementICPSLAMParametersProperty(this, "slamParametersProperty");

   public FrameNormalEstimationAnchorPaneController()
   {
   }

   private void setupControls()
   {
      searchRadiusSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxDistanceFromPlaneSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
   }

   private final Topic<Boolean> saveMainUpdaterConfigurationTopic = SLAMModuleAPI.SaveConfiguration;
   private final Topic<NormalEstimationParameters> normalEstimationParametersTopic = SLAMModuleAPI.FrameNormalEstimationParameters;


   @Override
   public void bindControls()
   {
      setupControls();

      slamParametersProperty.bindBidirectionalComputeSurfaceNormalsInFrame(enableButton.selectedProperty());

      normalEstimationParametersProperty.bindBidirectionalSearchRadius(searchRadiusSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMinConsensusRatio(minConsensusRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxAverageDeviationRatio(maxAverageDeviationRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalNumberOfIterations(numberOfIterationsSlider.valueProperty());
      normalEstimationParametersProperty.bindBidrectionalWeightByNumberOfHits(weightNumberOfHits.selectedProperty());
      normalEstimationParametersProperty.bindBidirectionalEnableLeastSquaresEstimation(enableLeastSquares.selectedProperty());

      uiMessager.bindBidirectionalGlobal(normalEstimationParametersTopic, normalEstimationParametersProperty);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(saveMainUpdaterConfigurationTopic);
   }
}
