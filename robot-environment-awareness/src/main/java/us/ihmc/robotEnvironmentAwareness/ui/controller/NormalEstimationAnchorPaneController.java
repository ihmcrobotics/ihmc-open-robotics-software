package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.NormalEstimationParametersProperty;

public class NormalEstimationAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;

   @FXML
   private ToggleButton weightNumberOfHits;

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

   private Topic<Boolean> normalEstimationEnableTopic = REAModuleAPI.NormalEstimationEnable;
   private Topic<Boolean> normalEstimationClearTopic = REAModuleAPI.NormalEstimationClear;
   private Topic<Boolean> saveMainUpdaterConfigurationTopic = REAModuleAPI.SaveMainUpdaterConfiguration;
   private Topic<NormalEstimationParameters> normalEstimationParametersTopic = REAModuleAPI.NormalEstimationParameters;

   public void setNormalEstimationEnableTopic(Topic<Boolean> normalEstimationEnableTopic)
   {
      this.normalEstimationEnableTopic = normalEstimationEnableTopic;
   }

   public void setNormalEstimationClearTopic(Topic<Boolean> normalEstimationClearTopic)
   {
      this.normalEstimationClearTopic = normalEstimationClearTopic;
   }

   public void setSaveMainUpdaterConfigurationTopic(Topic<Boolean> saveMainUpdaterConfigurationTopic)
   {
      this.saveMainUpdaterConfigurationTopic = saveMainUpdaterConfigurationTopic;
   }

   public void setNormalEstimationParametersTopic(Topic<NormalEstimationParameters> normalEstimationParametersTopic)
   {
      this.normalEstimationParametersTopic = normalEstimationParametersTopic;
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(normalEstimationEnableTopic, enableButton.selectedProperty());

      normalEstimationParametersProperty.bindBidirectionalSearchRadius(searchRadiusSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxDistanceFromPlane(maxDistanceFromPlaneSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMinConsensusRatio(minConsensusRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalMaxAverageDeviationRatio(maxAverageDeviationRatioSlider.valueProperty());
      normalEstimationParametersProperty.bindBidirectionalNumberOfIterations(numberOfIterationsSlider.valueProperty());
      normalEstimationParametersProperty.bindBidrectionalWeightByNumberOfHits(weightNumberOfHits.selectedProperty());

      uiMessager.bindBidirectionalGlobal(normalEstimationParametersTopic, normalEstimationParametersProperty);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(saveMainUpdaterConfigurationTopic);
   }

   @FXML
   public void resetNormals()
   {
      uiMessager.broadcastMessage(normalEstimationClearTopic, true);
   }
}
