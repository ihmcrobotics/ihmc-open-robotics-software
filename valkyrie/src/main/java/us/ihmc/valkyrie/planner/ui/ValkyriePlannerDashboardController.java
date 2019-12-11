package us.ihmc.valkyrie.planner.ui;

import javafx.animation.AnimationTimer;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.TextField;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlannerParameters;

import java.util.function.Consumer;

public class ValkyriePlannerDashboardController
{
   private Runnable doPlanningCallback = null;
   private Runnable haltPlanningCallback = null;
   private Runnable sendPlanningResultCallback = null;
   private Runnable stopWalkingCallback = null;
   private Runnable placeGoalCallback = null;
   private Consumer<DataSetName> dataSetSelectionCallback = null;
   private TimeElapsedManager timeElapsedManager = new TimeElapsedManager();

   @FXML
   private Spinner<Double> idealFootstepWidth;
   @FXML
   private Spinner<Double> minimumFootstepLength;
   @FXML
   private Spinner<Double> idealFootstepLength;
   @FXML
   private Spinner<Double> minimumStepWidth;
   @FXML
   private Spinner<Double> maximumStepWidth;
   @FXML
   private Spinner<Double> maximumStepReach;
   @FXML
   private Spinner<Double> minimumXClearanceFromStance;
   @FXML
   private Spinner<Double> minimumYClearanceFromStance;
   @FXML
   private Spinner<Double> minimumStepYaw;
   @FXML
   private Spinner<Double> maximumStepYaw;
   @FXML
   private Spinner<Double> stepYawReductionFactorAtMaxReach;
   @FXML
   private Spinner<Double> maximumStepZ;
   @FXML
   private Spinner<Double> minimumFootholdPercent;
   @FXML
   private Spinner<Double> maximumSurfanceInclineRadians;
   @FXML
   private Spinner<Double> wiggleInsideDelta;
   @FXML
   private Spinner<Double> maximumXYWiggle;
   @FXML
   private Spinner<Double> maximumYawWiggle;
   @FXML
   private Spinner<Double> cliffHeightToAvoid;
   @FXML
   private Spinner<Double> minimumDistanceFromCliffBottoms;
   @FXML
   private Spinner<Double> flatGroundLowerThreshold;
   @FXML
   private Spinner<Double> flatGroundUpperThreshold;
   @FXML
   private Spinner<Double> maximumStepWidthWhenSteppingDown;
   @FXML
   private Spinner<Double> maximumStepReachWhenSteppingDown;
   @FXML
   private Spinner<Double> maximumStepWidthWhenSteppingUp;
   @FXML
   private Spinner<Double> maximumStepReachWhenSteppingUp;
   @FXML
   private Spinner<Double> translationScaleFromGrandparentNode;
   @FXML
   private Spinner<Double> finalTurnProximity;
   @FXML
   private CheckBox checkForPathCollisions;
   @FXML
   private CheckBox checkForBodyBoxCollisions;
   @FXML
   private Spinner<Double> bodyBoxDimensionX;
   @FXML
   private Spinner<Double> bodyBoxDimensionY;
   @FXML
   private Spinner<Double> bodyBoxDimensionZ;
   @FXML
   private Spinner<Double> bodyBoxOffsetX;
   @FXML
   private Spinner<Double> bodyBoxOffsetY;
   @FXML
   private Spinner<Double> bodyBoxOffsetZ;
   @FXML
   private Spinner<Integer> numberOfBoundingBoxChecks;
   @FXML
   private Spinner<Double> translationWeightX;
   @FXML
   private Spinner<Double> translationWeightY;
   @FXML
   private Spinner<Double> translationWeightZ;
   @FXML
   private Spinner<Double> orientationWeightYaw;
   @FXML
   private Spinner<Double> orientationWeightPitch;
   @FXML
   private Spinner<Double> orientationWeightRoll;
   @FXML
   private Spinner<Double> costPerStep;
   @FXML
   private Spinner<Double> footholdAreaWeight;
   @FXML
   private Spinner<Double> astarHeuristicsWeight;

   @FXML
   private TextField timeElapsed;
   @FXML
   private ComboBox<DataSetName> dataSetSelector;
   @FXML
   private Spinner<Double> timeout;
   @FXML
   private Spinner<Double> goalX;
   @FXML
   private Spinner<Double> goalY;
   @FXML
   private Spinner<Double> goalZ;
   @FXML
   private Spinner<Double> goalYaw;

   @FXML
   public void doPlanning()
   {
      new Thread(doPlanningCallback).start();
      timeElapsedManager.start();
   }

   @FXML
   public void haltPlanning()
   {
      haltPlanningCallback.run();
   }

   @FXML
   public void sendPlanningResult()
   {
      sendPlanningResultCallback.run();
   }

   @FXML
   public void stopWalking()
   {
      stopWalkingCallback.run();
   }

   @FXML
   public void placeGoal()
   {
      placeGoalCallback.run();
   }

   public void setDoPlanningCallback(Runnable doPlanningCallback)
   {
      this.doPlanningCallback = doPlanningCallback;
   }

   public void onPlannerCompleted()
   {
      timeElapsedManager.stop();
   }

   public void setHaltPlanningCallback(Runnable haltPlanningCallback)
   {
      this.haltPlanningCallback = haltPlanningCallback;
   }

   public void setSendPlanningResultCallback(Runnable sendPlanningResultCallback)
   {
      this.sendPlanningResultCallback = sendPlanningResultCallback;
   }

   public void setStopWalkingCallback(Runnable stopWalkingCallback)
   {
      this.stopWalkingCallback = stopWalkingCallback;
   }

   public void setDataSetSelectionCallback(Consumer<DataSetName> dataSetSelectionCallback)
   {
      this.dataSetSelectionCallback = dataSetSelectionCallback;
   }

   public void setPlaceGoalCallback(Runnable placeGoalCallback)
   {
      this.placeGoalCallback = placeGoalCallback;
   }

   public void setParameters(ValkyrieAStarFootstepPlannerParameters parameters)
   {
      idealFootstepWidth.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumFootstepLength.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      idealFootstepLength.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumStepWidth.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepWidth.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepReach.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumXClearanceFromStance.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumYClearanceFromStance.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumStepYaw.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepYaw.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      stepYawReductionFactorAtMaxReach.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepZ.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumFootholdPercent.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumSurfanceInclineRadians.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      wiggleInsideDelta.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumXYWiggle.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumYawWiggle.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      cliffHeightToAvoid.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      minimumDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      flatGroundLowerThreshold.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      flatGroundUpperThreshold.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepWidthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepReachWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepWidthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      maximumStepReachWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      translationScaleFromGrandparentNode.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      finalTurnProximity.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      bodyBoxDimensionX.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      bodyBoxDimensionY.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      bodyBoxDimensionZ.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      bodyBoxOffsetX.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      bodyBoxOffsetY.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      bodyBoxOffsetZ.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      numberOfBoundingBoxChecks.setValueFactory(new IntegerSpinnerValueFactory(0, Integer.MAX_VALUE, 0, 1));
      translationWeightX.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      translationWeightY.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      translationWeightZ.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      orientationWeightYaw.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      orientationWeightPitch.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      orientationWeightRoll.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      costPerStep.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      footholdAreaWeight.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      astarHeuristicsWeight.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));

      timeout.setValueFactory(new DoubleSpinnerValueFactory(0.0, Double.MAX_VALUE, 15.0, 1.0));
      goalX.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalY.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalZ.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));
      goalYaw.setValueFactory(new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1));

      idealFootstepWidth.getValueFactory().setValue(parameters.getIdealFootstepWidth());
      minimumFootstepLength.getValueFactory().setValue(parameters.getMinimumFootstepLength());
      idealFootstepLength.getValueFactory().setValue(parameters.getIdealFootstepLength());
      minimumStepWidth.getValueFactory().setValue(parameters.getMinimumStepWidth());
      maximumStepWidth.getValueFactory().setValue(parameters.getMaximumStepWidth());
      maximumStepReach.getValueFactory().setValue(parameters.getMaximumStepReach());
      minimumXClearanceFromStance.getValueFactory().setValue(parameters.getMinimumXClearanceFromStance());
      minimumYClearanceFromStance.getValueFactory().setValue(parameters.getMinimumYClearanceFromStance());
      minimumStepYaw.getValueFactory().setValue(parameters.getMinimumStepYaw());
      maximumStepYaw.getValueFactory().setValue(parameters.getMaximumStepYaw());
      stepYawReductionFactorAtMaxReach.getValueFactory().setValue(parameters.getStepYawReductionFactorAtMaxReach());
      maximumStepZ.getValueFactory().setValue(parameters.getMaximumStepZ());
      minimumFootholdPercent.getValueFactory().setValue(parameters.getMinimumFootholdPercent());
      maximumSurfanceInclineRadians.getValueFactory().setValue(parameters.getMaximumSurfanceInclineRadians());
      wiggleInsideDelta.getValueFactory().setValue(parameters.getWiggleInsideDelta());
      maximumXYWiggle.getValueFactory().setValue(parameters.getMaximumXYWiggle());
      maximumYawWiggle.getValueFactory().setValue(parameters.getMaximumYawWiggle());
      cliffHeightToAvoid.getValueFactory().setValue(parameters.getCliffHeightToAvoid());
      minimumDistanceFromCliffBottoms.getValueFactory().setValue(parameters.getMinimumDistanceFromCliffBottoms());
      flatGroundLowerThreshold.getValueFactory().setValue(parameters.getFlatGroundLowerThreshold());
      flatGroundUpperThreshold.getValueFactory().setValue(parameters.getFlatGroundUpperThreshold());
      maximumStepWidthWhenSteppingDown.getValueFactory().setValue(parameters.getMaximumStepWidthWhenSteppingDown());
      maximumStepReachWhenSteppingDown.getValueFactory().setValue(parameters.getMaximumStepReachWhenSteppingDown());
      maximumStepWidthWhenSteppingUp.getValueFactory().setValue(parameters.getMaximumStepWidthWhenSteppingUp());
      maximumStepReachWhenSteppingUp.getValueFactory().setValue(parameters.getMaximumStepReachWhenSteppingUp());
      translationScaleFromGrandparentNode.getValueFactory().setValue(parameters.getTranslationScaleFromGrandparentNode());
      finalTurnProximity.getValueFactory().setValue(parameters.getFinalTurnProximity());
      checkForPathCollisions.setSelected(parameters.getCheckForPathCollisions());
      checkForBodyBoxCollisions.setSelected(parameters.getCheckForBodyBoxCollisions());
      bodyBoxDimensionX.getValueFactory().setValue(parameters.getBodyBoxDimensions().getX());
      bodyBoxDimensionY.getValueFactory().setValue(parameters.getBodyBoxDimensions().getY());
      bodyBoxDimensionZ.getValueFactory().setValue(parameters.getBodyBoxDimensions().getZ());
      bodyBoxOffsetX.getValueFactory().setValue(parameters.getBodyBoxDimensions().getX());
      bodyBoxOffsetY.getValueFactory().setValue(parameters.getBodyBoxDimensions().getY());
      bodyBoxOffsetZ.getValueFactory().setValue(parameters.getBodyBoxDimensions().getZ());
      numberOfBoundingBoxChecks.getValueFactory().setValue(parameters.getNumberOfBoundingBoxChecks());
      translationWeightX.getValueFactory().setValue(parameters.getTranslationWeight().getX());
      translationWeightY.getValueFactory().setValue(parameters.getTranslationWeight().getY());
      translationWeightZ.getValueFactory().setValue(parameters.getTranslationWeight().getZ());
      orientationWeightYaw.getValueFactory().setValue(parameters.getOrientationWeight().getYaw());
      orientationWeightPitch.getValueFactory().setValue(parameters.getOrientationWeight().getPitch());
      orientationWeightRoll.getValueFactory().setValue(parameters.getOrientationWeight().getRoll());
      costPerStep.getValueFactory().setValue(parameters.getCostPerStep());
      footholdAreaWeight.getValueFactory().setValue(parameters.getFootholdAreaWeight());
      astarHeuristicsWeight.getValueFactory().setValue(parameters.getAstarHeuristicsWeight());

      idealFootstepWidth.getValueFactory().valueProperty().addListener(observable -> parameters.setIdealFootstepWidth(idealFootstepWidth.getValue()));
      minimumFootstepLength.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumFootstepLength(minimumFootstepLength.getValue()));
      idealFootstepLength.getValueFactory().valueProperty().addListener(observable -> parameters.setIdealFootstepLength(idealFootstepLength.getValue()));
      minimumStepWidth.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumStepWidth(minimumStepWidth.getValue()));
      maximumStepWidth.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepWidth(maximumStepWidth.getValue()));
      maximumStepReach.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepReach(maximumStepReach.getValue()));
      minimumXClearanceFromStance.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumXClearanceFromStance(minimumXClearanceFromStance.getValue()));
      minimumYClearanceFromStance.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumYClearanceFromStance(minimumYClearanceFromStance.getValue()));
      minimumStepYaw.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumStepYaw(minimumStepYaw.getValue()));
      maximumStepYaw.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepYaw(maximumStepYaw.getValue()));
      stepYawReductionFactorAtMaxReach.getValueFactory().valueProperty().addListener(observable -> parameters.setStepYawReductionFactorAtMaxReach(stepYawReductionFactorAtMaxReach.getValue()));
      maximumStepZ.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepZ(maximumStepZ.getValue()));
      minimumFootholdPercent.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumFootholdPercent(minimumFootholdPercent.getValue()));
      maximumSurfanceInclineRadians.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumSurfanceInclineRadians(maximumSurfanceInclineRadians.getValue()));
      wiggleInsideDelta.getValueFactory().valueProperty().addListener(observable -> parameters.setWiggleInsideDelta(wiggleInsideDelta.getValue()));
      maximumXYWiggle.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumXYWiggle(maximumXYWiggle.getValue()));
      maximumYawWiggle.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumYawWiggle(maximumYawWiggle.getValue()));
      cliffHeightToAvoid.getValueFactory().valueProperty().addListener(observable -> parameters.setCliffHeightToAvoid(cliffHeightToAvoid.getValue()));
      minimumDistanceFromCliffBottoms.getValueFactory().valueProperty().addListener(observable -> parameters.setMinimumDistanceFromCliffBottoms(minimumDistanceFromCliffBottoms.getValue()));
      flatGroundLowerThreshold.getValueFactory().valueProperty().addListener(observable -> parameters.setFlatGroundLowerThreshold(flatGroundLowerThreshold.getValue()));
      flatGroundUpperThreshold.getValueFactory().valueProperty().addListener(observable -> parameters.setFlatGroundUpperThreshold(flatGroundUpperThreshold.getValue()));
      maximumStepWidthWhenSteppingDown.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepWidthWhenSteppingDown(maximumStepWidthWhenSteppingDown.getValue()));
      maximumStepReachWhenSteppingDown.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepReachWhenSteppingDown(maximumStepReachWhenSteppingDown.getValue()));
      maximumStepWidthWhenSteppingUp.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepWidthWhenSteppingUp(maximumStepWidthWhenSteppingUp.getValue()));
      maximumStepReachWhenSteppingUp.getValueFactory().valueProperty().addListener(observable -> parameters.setMaximumStepReachWhenSteppingUp(maximumStepReachWhenSteppingUp.getValue()));
      translationScaleFromGrandparentNode.getValueFactory().valueProperty().addListener(observable -> parameters.setTranslationScaleFromGrandparentNode(translationScaleFromGrandparentNode.getValue()));
      finalTurnProximity.getValueFactory().valueProperty().addListener(observable -> parameters.setFinalTurnProximity(finalTurnProximity.getValue()));
      checkForPathCollisions.selectedProperty().addListener(observable -> parameters.setCheckForPathCollisions(checkForPathCollisions.isSelected()));
      checkForBodyBoxCollisions.selectedProperty().addListener(observable -> parameters.setCheckForBodyBoxCollisions(checkForBodyBoxCollisions.isSelected()));
      bodyBoxDimensionX.getValueFactory().valueProperty().addListener(observable -> parameters.setBodyBoxDimensionX(bodyBoxDimensionX.getValue()));
      bodyBoxDimensionY.getValueFactory().valueProperty().addListener(observable -> parameters.setBodyBoxDimensionY(bodyBoxDimensionY.getValue()));
      bodyBoxDimensionZ.getValueFactory().valueProperty().addListener(observable -> parameters.setBodyBoxDimensionZ(bodyBoxDimensionZ.getValue()));
      bodyBoxOffsetX.getValueFactory().valueProperty().addListener(observable -> parameters.setBodyBoxOffsetX(bodyBoxOffsetX.getValue()));
      bodyBoxOffsetY.getValueFactory().valueProperty().addListener(observable -> parameters.setBodyBoxOffsetY(bodyBoxOffsetY.getValue()));
      bodyBoxOffsetZ.getValueFactory().valueProperty().addListener(observable -> parameters.setBodyBoxOffsetZ(bodyBoxOffsetZ.getValue()));
      numberOfBoundingBoxChecks.getValueFactory().valueProperty().addListener(observable -> parameters.setNumberOfBoundingBoxChecks(numberOfBoundingBoxChecks.getValue()));
      translationWeightX.getValueFactory().valueProperty().addListener(observable -> parameters.setTranslationWeightX(translationWeightX.getValue()));
      translationWeightY.getValueFactory().valueProperty().addListener(observable -> parameters.setTranslationWeightY(translationWeightY.getValue()));
      translationWeightZ.getValueFactory().valueProperty().addListener(observable -> parameters.setTranslationWeightZ(translationWeightZ.getValue()));
      orientationWeightYaw.getValueFactory().valueProperty().addListener(observable -> parameters.setOrientationWeightYaw(orientationWeightYaw.getValue()));
      orientationWeightPitch.getValueFactory().valueProperty().addListener(observable -> parameters.setOrientationWeightPitch(orientationWeightPitch.getValue()));
      orientationWeightRoll.getValueFactory().valueProperty().addListener(observable -> parameters.setOrientationWeightRoll(orientationWeightRoll.getValue()));
      costPerStep.getValueFactory().valueProperty().addListener(observable -> parameters.setCostPerStep(costPerStep.getValue()));
      footholdAreaWeight.getValueFactory().valueProperty().addListener(observable -> parameters.setFootholdAreaWeight(footholdAreaWeight.getValue()));
      astarHeuristicsWeight.getValueFactory().valueProperty().addListener(observable -> parameters.setAstarHeuristicsWeight(astarHeuristicsWeight.getValue()));

      dataSetSelector.getItems().setAll(DataSetName.values());
      dataSetSelector.getSelectionModel().selectedItemProperty().addListener((options, oldValue, newValue) -> dataSetSelectionCallback.accept(dataSetSelector.getValue()));
   }

   private class TimeElapsedManager extends AnimationTimer
   {
      private final Stopwatch stopwatch = new Stopwatch();

      @Override
      public void start()
      {
         stopwatch.start();
         super.start();
      }

      @Override
      public void handle(long now)
      {
         timeElapsed.setText(String.format("%.2f", stopwatch.totalElapsed()));
      }

      @Override
      public void stop()
      {
         stopwatch.suspend();
      }
   }

   public Spinner<Double> getGoalX()
   {
      return goalX;
   }

   public Spinner<Double> getGoalY()
   {
      return goalY;
   }

   public Spinner<Double> getGoalZ()
   {
      return goalZ;
   }

   public Spinner<Double> getGoalYaw()
   {
      return goalYaw;
   }

   public double getTimeout()
   {
      return timeout.getValue();
   }
}
