package us.ihmc.robotEnvironmentAwareness.ui.controller;

import controller_msgs.msg.dds.ConcaveHullFactoryParametersMessage;
import controller_msgs.msg.dds.ConcaveHullFactoryParametersStringMessage;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import perception_msgs.msg.dds.PolygonizerParametersMessage;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAParametersMessageHelper;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ConcaveHullFactoryParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.IntersectionEstimationParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PolygonizerParametersProperty;

public class PolygonizerAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enablePolygonizerButton;
   @FXML
   private ToggleButton enableIntersectionCalculatorButton;
   @FXML
   private ToggleButton hideRegionNodes;

   // Polygonizer parameters
   @FXML
   private Spinner<Double> concaveHullThresholdSpinner;
   @FXML
   private Spinner<Integer> minRegionSizePolygonizerSpinner;
   @FXML
   private Spinner<Double> peakAngleThresholdSpinner;
   @FXML
   private Spinner<Double> shallowAngleThresholdSpinner;
   @FXML
   private Spinner<Double> minEdgeLengthSpinner;
   @FXML
   private Spinner<Double> depthThresholdSpinner;
   @FXML
   private ToggleButton enableNarrowPassageFilterButton;

   private final ConcaveHullFactoryParametersProperty concaveHullFactoryParametersProperty = new ConcaveHullFactoryParametersProperty(this, "concaveHullParameters");
   private final PolygonizerParametersProperty polygonizerParametersProperty = new PolygonizerParametersProperty(this, "polygonizerParameters");

   // Intersection calculator parameters
   @FXML
   private Spinner<Double> maxDistanceToRegionSpinner;
   @FXML
   private Spinner<Integer> minRegionSizeIntersectionSpinner;
   @FXML
   private Spinner<Double> minIntersectionLengthSpinner;
   @FXML
   private Spinner<Double> minRegionAngleDifferenceSpinner;
   @FXML
   private ToggleButton addIntersectionsToRegionsButton;

   private final IntersectionEstimationParametersProperty intersectionEstimationParametersProperty = new IntersectionEstimationParametersProperty(this, "intersectionEstimationParameters");

   public PolygonizerAnchorPaneController()
   {
   }

   private void setupControls()
   {
      concaveHullThresholdSpinner.setValueFactory(createLengthValueFactory(0.001, 0.50, 0.2, 0.05));
      minRegionSizePolygonizerSpinner.setValueFactory(new IntegerSpinnerValueFactory(0, 1000, 10, 10));
      peakAngleThresholdSpinner.setValueFactory(createAngleValueFactory(Math.PI / 2.0, Math.PI, Math.toRadians(160), Math.toRadians(5.0)));
      shallowAngleThresholdSpinner.setValueFactory(createAngleValueFactory(0.0, Math.PI / 2.0, Math.toRadians(10), Math.toRadians(2.5)));
      minEdgeLengthSpinner.setValueFactory(createLengthValueFactory(0.0, 0.20, 0.05, 0.005));
      depthThresholdSpinner.setValueFactory(createLengthValueFactory(0.001, 0.50, 0.10, 0.05));

      maxDistanceToRegionSpinner.setValueFactory(createLengthValueFactory(0.0, 0.50, 0.05, 0.01));
      minRegionSizeIntersectionSpinner.setValueFactory(new IntegerSpinnerValueFactory(0, 1000, 10, 10));
      minIntersectionLengthSpinner.setValueFactory(createLengthValueFactory(0.0, 0.50, 0.06, 0.01));
      minRegionAngleDifferenceSpinner.setValueFactory(createAngleValueFactory(0.0, Math.PI / 2.0, Math.toRadians(15.0), Math.toRadians(5.0)));

      concaveHullThresholdSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      peakAngleThresholdSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
      shallowAngleThresholdSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
      minEdgeLengthSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      depthThresholdSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());

      maxDistanceToRegionSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      minIntersectionLengthSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      minRegionAngleDifferenceSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
   }

   private Topic<Boolean> planarRegionsPolygonizerEnableTopic = REAModuleAPI.PlanarRegionsPolygonizerEnable;
   private Topic<Boolean> planarRegionsPolygonizerClearTopic = REAModuleAPI.PlanarRegionsPolygonizerClear;
   private Topic<Boolean> planarRegionsIntersectionEnableTopic = REAModuleAPI.PlanarRegionsIntersectionEnable;
   private Topic<Boolean> uiPlanarRegionHideNodesTopic = REAModuleAPI.UIPlanarRegionHideNodes;
   private Topic<Boolean> saveRegionUpdaterConfigurationTopic = REAModuleAPI.SaveRegionUpdaterConfiguration;
   private Topic<ConcaveHullFactoryParametersMessage> planarRegionsConcaveHullParametersTopic = REAModuleAPI.PlanarRegionsConcaveHullParameters;
   private Topic<PolygonizerParametersMessage> planarRegionsPolygonizerParametersTopic = REAModuleAPI.PlanarRegionsPolygonizerParameters;
   private Topic<IntersectionEstimationParameters> planarRegionsIntersectionParametersTopic = REAModuleAPI.PlanarRegionsIntersectionParameters;

   public void setPlanarRegionsPolygonizerEnableTopic(Topic<Boolean> planarRegionsPolygonizerEnableTopic)
   {
      this.planarRegionsPolygonizerEnableTopic = planarRegionsPolygonizerEnableTopic;
   }

   public void setPlanarRegionsPolygonizerClearTopic(Topic<Boolean> planarRegionsPolygonizerClearTopic)
   {
      this.planarRegionsPolygonizerClearTopic = planarRegionsPolygonizerClearTopic;
   }

   public void setPlanarRegionsIntersectionEnableTopic(Topic<Boolean> planarRegionsIntersectionEnableTopic)
   {
      this.planarRegionsIntersectionEnableTopic = planarRegionsIntersectionEnableTopic;
   }

   public void setUiPlanarRegionHideNodesTopic(Topic<Boolean> uiPlanarRegionHideNodesTopic)
   {
      this.uiPlanarRegionHideNodesTopic = uiPlanarRegionHideNodesTopic;
   }

   public void setSaveRegionUpdaterConfigurationTopic(Topic<Boolean> saveRegionUpdaterConfigurationTopic)
   {
      this.saveRegionUpdaterConfigurationTopic = saveRegionUpdaterConfigurationTopic;
   }

   public void setPlanarRegionsConcaveHullParametersTopic(Topic<ConcaveHullFactoryParametersMessage> planarRegionsConcaveHullParametersTopic)
   {
      this.planarRegionsConcaveHullParametersTopic = planarRegionsConcaveHullParametersTopic;
   }

   public void setPlanarRegionsPolygonizerParametersTopic(Topic<PolygonizerParametersMessage> planarRegionsPolygonizerParametersTopic)
   {
      this.planarRegionsPolygonizerParametersTopic = planarRegionsPolygonizerParametersTopic;
   }

   public void setPlanarRegionsIntersectionParametersTopic(Topic<IntersectionEstimationParameters> planarRegionsIntersectionParametersTopic)
   {
      this.planarRegionsIntersectionParametersTopic = planarRegionsIntersectionParametersTopic;
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(planarRegionsPolygonizerEnableTopic, enablePolygonizerButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(planarRegionsIntersectionEnableTopic, enableIntersectionCalculatorButton.selectedProperty());

      concaveHullFactoryParametersProperty.bindBidirectionalEdgeLengthThreshold(concaveHullThresholdSpinner.getValueFactory().valueProperty());

      uiMessager.registerTopicListener(planarRegionsConcaveHullParametersTopic, message -> concaveHullFactoryParametersProperty.set(REAParametersMessageHelper.convertFromMessage(message)));
      concaveHullFactoryParametersProperty.addListener((obs, oldValue, newValue) -> uiMessager.submitMessageToModule(planarRegionsConcaveHullParametersTopic, REAParametersMessageHelper.convertToMessage(newValue)));

      uiMessager.registerTopicListener(planarRegionsPolygonizerParametersTopic, parametersMessage -> polygonizerParametersProperty.set(REAParametersMessageHelper.convertFromMessage(parametersMessage)));
      polygonizerParametersProperty.addListener((obs, oldValue, newValue) -> uiMessager.submitMessageToModule(planarRegionsPolygonizerParametersTopic, REAParametersMessageHelper.convertToMessage(newValue)));

      polygonizerParametersProperty.bindBidirectionalMinNumberOfNodes(minRegionSizePolygonizerSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalPeakAngleThreshold(peakAngleThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalShallowAngleThreshold(shallowAngleThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalLengthThreshold(minEdgeLengthSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalDepthThreshold(depthThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalEnableNarrowPassageFilter(enableNarrowPassageFilterButton.selectedProperty());

      intersectionEstimationParametersProperty.bindBidirectionalMaxDistanceToRegion(maxDistanceToRegionSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinRegionSize(minRegionSizeIntersectionSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinIntersectionLength(minIntersectionLengthSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinRegionAngleDifference(minRegionAngleDifferenceSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalAddIntersectionsToRegions(addIntersectionsToRegionsButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(planarRegionsIntersectionParametersTopic, intersectionEstimationParametersProperty);
      
      load();
      uiMessager.bindBidirectionalInternal(uiPlanarRegionHideNodesTopic, hideRegionNodes.selectedProperty(), true);
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(planarRegionsPolygonizerClearTopic, true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(saveRegionUpdaterConfigurationTopic);
      saveUIControlProperty(uiPlanarRegionHideNodesTopic, hideRegionNodes);
   }

   public void load()
   {
      loadUIControlProperty(uiPlanarRegionHideNodesTopic, hideRegionNodes);
   }

   public static DoubleSpinnerValueFactory createLengthValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.metersToRoundedCentimeters());
      return doubleSpinnerValueFactory;
   }

   public static DoubleSpinnerValueFactory createAngleValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.radiansToRoundedDegrees());
      return doubleSpinnerValueFactory;
   }
}
