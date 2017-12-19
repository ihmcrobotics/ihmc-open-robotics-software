package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
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

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.PlanarRegionsPolygonizerEnable, enablePolygonizerButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.PlanarRegionsIntersectionEnable, enableIntersectionCalculatorButton.selectedProperty());

      concaveHullFactoryParametersProperty.bindBidirectionalEdgeLengthThreshold(concaveHullThresholdSpinner.getValueFactory().valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.PlanarRegionsConcaveHullParameters, concaveHullFactoryParametersProperty);

      polygonizerParametersProperty.bindBidirectionalMinNumberOfNodes(minRegionSizePolygonizerSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalPeakAngleThreshold(peakAngleThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalShallowAngleThreshold(shallowAngleThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalLengthThreshold(minEdgeLengthSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalDepthThreshold(depthThresholdSpinner.getValueFactory().valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.PlanarRegionsPolygonizerParameters, polygonizerParametersProperty);

      intersectionEstimationParametersProperty.bindBidirectionalMaxDistanceToRegion(maxDistanceToRegionSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinRegionSize(minRegionSizeIntersectionSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinIntersectionLength(minIntersectionLengthSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinRegionAngleDifference(minRegionAngleDifferenceSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalAddIntersectionsToRegions(addIntersectionsToRegionsButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.PlanarRegionsIntersectionParameters, intersectionEstimationParametersProperty);
      
      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIPlanarRegionHideNodes, hideRegionNodes.selectedProperty(), true);
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(REAModuleAPI.PlanarRegionsPolygonizerClear, true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveRegionUpdaterConfiguration);
      saveUIControlProperty(REAModuleAPI.UIPlanarRegionHideNodes, hideRegionNodes);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UIPlanarRegionHideNodes, hideRegionNodes);
   }

   private DoubleSpinnerValueFactory createLengthValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.metersToRoundedCentimeters());
      return doubleSpinnerValueFactory;
   }

   private DoubleSpinnerValueFactory createAngleValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.radiansToRoundedDegrees());
      return doubleSpinnerValueFactory;
   }
}
