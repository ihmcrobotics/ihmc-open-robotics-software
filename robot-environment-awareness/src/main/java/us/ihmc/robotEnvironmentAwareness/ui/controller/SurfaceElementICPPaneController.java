package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceElementICPSLAMParametersProperty;

public class SurfaceElementICPPaneController extends REABasicUIController
{
   @FXML
   private Spinner<Double> surfaceElementResolution;
   
   @FXML
   private Spinner<Double> windowMargin;
   
   @FXML
   private Spinner<Integer> minimumNumberOfHit;
   
   @FXML
   private Spinner<Double> boundRatio;
   
   @FXML
   private Spinner<Integer> steadyStateDetectorIterationThreshold;
   
   @FXML
   private Spinner<Double> qualityConvergenceThreshold;
   
   @FXML
   private Spinner<Double> translationalEffortConvergenceThreshold;
   
   @FXML
   private Spinner<Double> rotationalEffortConvergenceThreshold;
   
   @FXML
   private ToggleButton enableInitialQualityFilter;
   
   @FXML
   private Spinner<Double> initialQualityThreshold;
   @FXML
   private Spinner<Integer> maxQueueSize;
   @FXML
   private Spinner<Double> longestTimeToLag;

   @FXML
   private Spinner<Integer> maxOptimizationIterations;
   @FXML
   private ToggleButton computeSurfaceNormalsInFrame;
   @FXML
   private ToggleButton insertMissInOcTree;
   @FXML
   private ToggleButton computeFramesInParallel;

   @FXML
   private ToggleButton includePitchAndRoll;
   @FXML
   private Spinner<Double> translationPerturbation;
   @FXML
   private Spinner<Double> rotationPerturbation;
   
   private final SurfaceElementICPSLAMParametersProperty surfaceElementICPSLAMParametersProperty = new SurfaceElementICPSLAMParametersProperty(this, "surfaceElementICPSLAMParametersProperty");
   
   private void setupControls()
   {
      surfaceElementResolution.setValueFactory(createDoubleValueFactory(0.03, 0.05, 0.04, 0.005));
      windowMargin.setValueFactory(createDoubleValueFactory(0.0, 0.1, 0.01, 0.01));
      minimumNumberOfHit.setValueFactory(createIntegerValueFactory(1, 10, 1, 1));
      boundRatio.setValueFactory(createDoubleValueFactory(1.05, 1.5, 1.1, 0.05));
      
      steadyStateDetectorIterationThreshold.setValueFactory(createIntegerValueFactory(3, 6, 3, 1));
      qualityConvergenceThreshold.setValueFactory(createDoubleValueFactory(0.0001, 0.002, 0.001, 0.0001));
      translationalEffortConvergenceThreshold.setValueFactory(createDoubleValueFactory(0.0001, 0.002, 0.001, 0.0001));
      rotationalEffortConvergenceThreshold.setValueFactory(createDoubleValueFactory(0.0004, 0.002, 0.005, 0.0001));
      
      initialQualityThreshold.setValueFactory(createDoubleValueFactory(0.05, 0.3, 0.1, 0.05));

      longestTimeToLag.setValueFactory(createDoubleValueFactory(0.0, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 1.0));
      maxOptimizationIterations.setValueFactory(createIntegerValueFactory(1, 100, 40, 1));
      translationPerturbation.setValueFactory(createDoubleValueFactory(0.0001, 0.01, 0.002, 0.0005));
      rotationPerturbation.setValueFactory(createDoubleValueFactory(0.00001, 0.0001, 0.00001, 0.00001));

      maxQueueSize.setValueFactory(createIntegerValueFactory(1, Integer.MAX_VALUE, 100, 5));
   }
   
   @Override
   public void bindControls()
   {
      setupControls();
      surfaceElementICPSLAMParametersProperty.bindBidirectionalSurfaceElementResolution(surfaceElementResolution.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalWindowMargin(windowMargin.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalMinimumNumberOfHit(minimumNumberOfHit.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalBoundRatio(boundRatio.getValueFactory().valueProperty());
      
      surfaceElementICPSLAMParametersProperty.bindBidirectionalSteadyStateDetectorIterationThreshold(steadyStateDetectorIterationThreshold.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalQualityConvergenceThreshold(qualityConvergenceThreshold.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalTranslationalEffortConvergenceThreshold(translationalEffortConvergenceThreshold.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalRotationalEffortConvergenceThreshold(rotationalEffortConvergenceThreshold.getValueFactory().valueProperty());
      
      surfaceElementICPSLAMParametersProperty.bindBidirectionalEnableInitialQualityFilter(enableInitialQualityFilter.selectedProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalInitialQualityThreshold(initialQualityThreshold.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalMaxQueueSize(maxQueueSize.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalLongestTimeToLag(longestTimeToLag.getValueFactory().valueProperty());

      surfaceElementICPSLAMParametersProperty.bindBidirectionalMaxOptimizationIterations(maxOptimizationIterations.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalComputeSurfaceNormalsInFrame(computeSurfaceNormalsInFrame.selectedProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalInsertMissInOcTree(insertMissInOcTree.selectedProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalComputeFramesInParallel(computeFramesInParallel.selectedProperty());

      surfaceElementICPSLAMParametersProperty.bindBidirectionalIncludePitchAndRoll(includePitchAndRoll.selectedProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalTranslationPerturbation(translationPerturbation.getValueFactory().valueProperty());
      surfaceElementICPSLAMParametersProperty.bindBidirectionalRotationPerturbation(rotationPerturbation.getValueFactory().valueProperty());
      
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SLAMParameters, surfaceElementICPSLAMParametersProperty);
   }

   public static DoubleSpinnerValueFactory createDoubleValueFactory(double min, double max, double initialValue, double amountToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.rounding(1.0, 4));
      return doubleSpinnerValueFactory;
   }
   
   public static IntegerSpinnerValueFactory createIntegerValueFactory(int min, int max, int initialValue, int amountToStepBy)
   {
      IntegerSpinnerValueFactory doubleSpinnerValueFactory = new IntegerSpinnerValueFactory(min, max, initialValue, amountToStepBy);
      return doubleSpinnerValueFactory;
   }
}
