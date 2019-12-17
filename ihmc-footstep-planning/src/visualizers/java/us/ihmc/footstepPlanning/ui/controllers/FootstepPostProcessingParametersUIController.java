package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingKeys;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

public class FootstepPostProcessingParametersUIController
{
   private JavaFXMessager messager;
   private FootstepPostProcessingParametersBasics postProcessingParameters;


   @FXML
   private CheckBox positionSplitFractionProcessingEnabled;
   @FXML
   private Spinner<Double> stepHeightForLargeStepDown;
   @FXML
   private Spinner<Double> largestStepDownHeight;
   @FXML
   private Spinner<Double> transferSplitFractionAtFullDepth;
   @FXML
   private Spinner<Double> transferWeightDistributionAtFullDepth;

   @FXML
   private CheckBox areaSplitFractionProcessingEnabled;
   @FXML
   private Spinner<Double> fractionLoadIfFootHasFullSupport;
   @FXML
   private Spinner<Double> fractionTimeOnFootIfFootHasFullSupport;
   @FXML
   private Spinner<Double> fractionLoadIfOtherFootHasNoWidth;
   @FXML
   private Spinner<Double> fractionTimeOnFootIfOtherFootHasNoWidth;

   @FXML
   private CheckBox swingOverRegionsProcessingEnabled;
   @FXML
   private CheckBox doInitialFastApproximation;
   @FXML
   private Spinner<Double> minimumSwingFootClearance;
   @FXML
   private Spinner<Integer> numberOfChecksPerSwing;
   @FXML
   private Spinner<Integer> maximumNumberOfAdjustmentAttempts;
   @FXML
   private Spinner<Double> maximumAdjustmentDistance;
   @FXML
   private Spinner<Double> incrementalAdjustmentDistance;
   @FXML
   private Spinner<Double> minimumHeightAboveFloorForCollision;

   public FootstepPostProcessingParametersUIController()
   {
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPostProcessingParameters(FootstepPostProcessingParametersBasics parameters)
   {
      this.postProcessingParameters = parameters;
   }

   public void setupControls()
   {
      stepHeightForLargeStepDown.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.15, 0.05));
      largestStepDownHeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.35, 0.05));
      transferSplitFractionAtFullDepth.setValueFactory(new DoubleSpinnerValueFactory(0.01, 0.99, 0.2, 0.05));
      transferWeightDistributionAtFullDepth.setValueFactory(new DoubleSpinnerValueFactory(0.01, 0.99, 0.7, 0.05));

      minimumSwingFootClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.04, 0.01));
      numberOfChecksPerSwing.setValueFactory(new IntegerSpinnerValueFactory(10, 200, 100, 10));
      maximumNumberOfAdjustmentAttempts.setValueFactory(new IntegerSpinnerValueFactory(5, 100, 50, 5));
      maximumAdjustmentDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.2, 0.05));
      incrementalAdjustmentDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.1, 0.03, 0.01));
      minimumHeightAboveFloorForCollision.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.15, 0.03, 0.01));

      fractionLoadIfFootHasFullSupport.setValueFactory(new DoubleSpinnerValueFactory(0.01, 0.99, 0.5, 0.05));
      fractionTimeOnFootIfFootHasFullSupport.setValueFactory(new DoubleSpinnerValueFactory(0.01, 0.99, 0.5, 0.05));
      fractionLoadIfOtherFootHasNoWidth.setValueFactory(new DoubleSpinnerValueFactory(0.01, 0.99, 0.5, 0.05));
      fractionTimeOnFootIfOtherFootHasNoWidth.setValueFactory(new DoubleSpinnerValueFactory(0.01, 0.99, 0.5, 0.05));
   }

   public void bindControls()
   {
      setupControls();

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(postProcessingParameters);
      javaFXStoredPropertyMap.put(positionSplitFractionProcessingEnabled, FootstepPostProcessingKeys.positionSplitFractionProcessingEnabled);
      javaFXStoredPropertyMap.put(stepHeightForLargeStepDown, FootstepPostProcessingKeys.stepHeightForLargeStepDown);
      javaFXStoredPropertyMap.put(largestStepDownHeight, FootstepPostProcessingKeys.largestStepDownHeight);
      javaFXStoredPropertyMap.put(transferSplitFractionAtFullDepth, FootstepPostProcessingKeys.transferSplitFractionAtFullDepth);
      javaFXStoredPropertyMap.put(transferWeightDistributionAtFullDepth, FootstepPostProcessingKeys.transferWeightDistributionAtFullDepth);

      javaFXStoredPropertyMap.put(swingOverRegionsProcessingEnabled, FootstepPostProcessingKeys.swingOverRegionsProcessingEnabled);
      javaFXStoredPropertyMap.put(doInitialFastApproximation, FootstepPostProcessingKeys.doInitialFastApproximation);
      javaFXStoredPropertyMap.put(minimumSwingFootClearance, FootstepPostProcessingKeys.minimumSwingFootClearance);
      javaFXStoredPropertyMap.put(numberOfChecksPerSwing, FootstepPostProcessingKeys.numberOfChecksPerSwing);
      javaFXStoredPropertyMap.put(maximumNumberOfAdjustmentAttempts, FootstepPostProcessingKeys.maximumNumberOfAdjustmentAttempts);
      javaFXStoredPropertyMap.put(maximumAdjustmentDistance, FootstepPostProcessingKeys.maximumWaypointAdjustmentDistance);
      javaFXStoredPropertyMap.put(incrementalAdjustmentDistance, FootstepPostProcessingKeys.incrementalWaypointAdjustmentDistance);
      javaFXStoredPropertyMap.put(minimumHeightAboveFloorForCollision, FootstepPostProcessingKeys.minimumHeightAboveFloorForCollision);

      javaFXStoredPropertyMap.put(areaSplitFractionProcessingEnabled, FootstepPostProcessingKeys.areaSplitFractionProcessingEnabled);
      javaFXStoredPropertyMap.put(fractionLoadIfFootHasFullSupport, FootstepPostProcessingKeys.fractionLoadIfFootHasFullSupport);
      javaFXStoredPropertyMap.put(fractionTimeOnFootIfFootHasFullSupport, FootstepPostProcessingKeys.fractionTimeOnFootIfFootHasFullSupport);
      javaFXStoredPropertyMap.put(fractionLoadIfOtherFootHasNoWidth, FootstepPostProcessingKeys.fractionLoadIfOtherFootHasNoWidth);
      javaFXStoredPropertyMap.put(fractionTimeOnFootIfOtherFootHasNoWidth, FootstepPostProcessingKeys.fractionTimeOnFootIfOtherFootHasNoWidth);

      // set messager updates to update all stored properties and select JavaFX properties
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PostProcessingParametersTopic, parameters ->
      {
         postProcessingParameters.set(parameters);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      // set JavaFX user input to update stored properties and publish messager message
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(() -> publishParameters());
   }

   private void publishParameters()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.PostProcessingParametersTopic, postProcessingParameters);
   }


   @FXML
   public void saveToFile()
   {
      postProcessingParameters.save();
   }
}
