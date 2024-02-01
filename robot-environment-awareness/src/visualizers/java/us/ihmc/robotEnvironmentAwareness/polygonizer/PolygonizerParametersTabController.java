package us.ihmc.robotEnvironmentAwareness.polygonizer;

import static us.ihmc.robotEnvironmentAwareness.ui.controller.PolygonizerAnchorPaneController.createAngleValueFactory;
import static us.ihmc.robotEnvironmentAwareness.ui.controller.PolygonizerAnchorPaneController.createLengthValueFactory;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ConcaveHullFactoryParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.IntersectionEstimationParametersProperty;
import us.ihmc.robotEnvironmentAwareness.ui.properties.PolygonizerParametersProperty;

public class PolygonizerParametersTabController
{
   // Polygonizer parameters
   @FXML
   private Spinner<Double> concaveHullThresholdSpinner;
   @FXML
   private Spinner<Integer> maxIterationsSpinner;
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

   private final ConcaveHullFactoryParametersProperty concaveHullFactoryParametersProperty = new ConcaveHullFactoryParametersProperty(this,
                                                                                                                                      "concaveHullParameters");
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

   private final IntersectionEstimationParametersProperty intersectionEstimationParametersProperty = new IntersectionEstimationParametersProperty(this,
                                                                                                                                                  "intersectionEstimationParameters");
   private JavaFXMessager messager;

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;
      setupControls();

      concaveHullFactoryParametersProperty.bindBidirectionalEdgeLengthThreshold(concaveHullThresholdSpinner.getValueFactory().valueProperty());
      concaveHullFactoryParametersProperty.bindBidirectionalMaxNumberOfIterations(maxIterationsSpinner.getValueFactory().valueProperty());
      messager.bindBidirectional(Polygonizer.PolygonizerParameters, concaveHullFactoryParametersProperty, false);

      polygonizerParametersProperty.bindBidirectionalMinNumberOfNodes(minRegionSizePolygonizerSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalPeakAngleThreshold(peakAngleThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalShallowAngleThreshold(shallowAngleThresholdSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalLengthThreshold(minEdgeLengthSpinner.getValueFactory().valueProperty());
      polygonizerParametersProperty.bindBidirectionalDepthThreshold(depthThresholdSpinner.getValueFactory().valueProperty());
      //      messager.bindBidirectional(REAModuleAPI.PlanarRegionsPolygonizerParameters, polygonizerParametersProperty, false); // TODO

      intersectionEstimationParametersProperty.bindBidirectionalMaxDistanceToRegion(maxDistanceToRegionSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinRegionSize(minRegionSizeIntersectionSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinIntersectionLength(minIntersectionLengthSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalMinRegionAngleDifference(minRegionAngleDifferenceSpinner.getValueFactory().valueProperty());
      intersectionEstimationParametersProperty.bindBidirectionalAddIntersectionsToRegions(addIntersectionsToRegionsButton.selectedProperty());
      //      messager.bindBidirectional(REAModuleAPI.PlanarRegionsIntersectionParameters, intersectionEstimationParametersProperty, false); // TODO

   }

   private void setupControls()
   {
      concaveHullThresholdSpinner.setValueFactory(createLengthValueFactory(0.001, 0.50, 0.2, 0.005));
      maxIterationsSpinner.setValueFactory(new IntegerSpinnerValueFactory(1, 5000, 5000, 1));

      minRegionSizePolygonizerSpinner.setValueFactory(new IntegerSpinnerValueFactory(0, 1000, 10, 10));
      peakAngleThresholdSpinner.setValueFactory(createAngleValueFactory(Math.PI / 2.0, Math.PI, Math.toRadians(160), Math.toRadians(5.0)));
      shallowAngleThresholdSpinner.setValueFactory(createAngleValueFactory(0.0, Math.PI / 2.0, Math.toRadians(10), Math.toRadians(2.5)));
      minEdgeLengthSpinner.setValueFactory(createLengthValueFactory(0.0, 0.20, 0.05, 0.005));
      depthThresholdSpinner.setValueFactory(createLengthValueFactory(0.001, 0.50, 0.10, 0.05));

      maxDistanceToRegionSpinner.setValueFactory(createLengthValueFactory(0.0, 0.50, 0.05, 0.01));
      minRegionSizeIntersectionSpinner.setValueFactory(new IntegerSpinnerValueFactory(0, 1000, 10, 10));
      minIntersectionLengthSpinner.setValueFactory(createLengthValueFactory(0.0, 0.50, 0.06, 0.01));
      minRegionAngleDifferenceSpinner.setValueFactory(createAngleValueFactory(0.0, Math.PI / 2.0, Math.toRadians(15.0), Math.toRadians(5.0)));

      concaveHullThresholdSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedMillimeters());
      peakAngleThresholdSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
      shallowAngleThresholdSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
      minEdgeLengthSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      depthThresholdSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());

      maxDistanceToRegionSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      minIntersectionLengthSpinner.getValueFactory().setConverter(StringConverterTools.metersToRoundedCentimeters());
      minRegionAngleDifferenceSpinner.getValueFactory().setConverter(StringConverterTools.radiansToRoundedDegrees());
   }

   @FXML
   public void reloadData()
   {
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationReload, true);
   }
}
