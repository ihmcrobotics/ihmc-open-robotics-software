package us.ihmc.valkyrie.planner.ui;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlannerParameters;

import java.util.function.Supplier;

public class ValkyriePlannerParametersController
{
   private ValkyrieAStarFootstepPlanner planner = null;

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

   void setPlanner(ValkyrieAStarFootstepPlanner planner)
   {
      this.planner = planner;
   }

   public void bindParameters()
   {
      Supplier<DoubleSpinnerValueFactory> doubleSpinnerValueFactory = () -> new DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1);
      idealFootstepWidth.setValueFactory(doubleSpinnerValueFactory.get());
      minimumFootstepLength.setValueFactory(doubleSpinnerValueFactory.get());
      idealFootstepLength.setValueFactory(doubleSpinnerValueFactory.get());
      minimumStepWidth.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepWidth.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepReach.setValueFactory(doubleSpinnerValueFactory.get());
      minimumXClearanceFromStance.setValueFactory(doubleSpinnerValueFactory.get());
      minimumYClearanceFromStance.setValueFactory(doubleSpinnerValueFactory.get());
      minimumStepYaw.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepYaw.setValueFactory(doubleSpinnerValueFactory.get());
      stepYawReductionFactorAtMaxReach.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepZ.setValueFactory(doubleSpinnerValueFactory.get());
      minimumFootholdPercent.setValueFactory(doubleSpinnerValueFactory.get());
      maximumSurfanceInclineRadians.setValueFactory(doubleSpinnerValueFactory.get());
      wiggleInsideDelta.setValueFactory(doubleSpinnerValueFactory.get());
      maximumXYWiggle.setValueFactory(doubleSpinnerValueFactory.get());
      maximumYawWiggle.setValueFactory(doubleSpinnerValueFactory.get());
      cliffHeightToAvoid.setValueFactory(doubleSpinnerValueFactory.get());
      minimumDistanceFromCliffBottoms.setValueFactory(doubleSpinnerValueFactory.get());
      flatGroundLowerThreshold.setValueFactory(doubleSpinnerValueFactory.get());
      flatGroundUpperThreshold.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepWidthWhenSteppingDown.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepReachWhenSteppingDown.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepWidthWhenSteppingUp.setValueFactory(doubleSpinnerValueFactory.get());
      maximumStepReachWhenSteppingUp.setValueFactory(doubleSpinnerValueFactory.get());
      translationScaleFromGrandparentNode.setValueFactory(doubleSpinnerValueFactory.get());
      finalTurnProximity.setValueFactory(doubleSpinnerValueFactory.get());
      bodyBoxDimensionX.setValueFactory(doubleSpinnerValueFactory.get());
      bodyBoxDimensionY.setValueFactory(doubleSpinnerValueFactory.get());
      bodyBoxDimensionZ.setValueFactory(doubleSpinnerValueFactory.get());
      bodyBoxOffsetX.setValueFactory(doubleSpinnerValueFactory.get());
      bodyBoxOffsetY.setValueFactory(doubleSpinnerValueFactory.get());
      bodyBoxOffsetZ.setValueFactory(doubleSpinnerValueFactory.get());
      numberOfBoundingBoxChecks.setValueFactory(new IntegerSpinnerValueFactory(0, Integer.MAX_VALUE, 0, 1));
      translationWeightX.setValueFactory(doubleSpinnerValueFactory.get());
      translationWeightY.setValueFactory(doubleSpinnerValueFactory.get());
      translationWeightZ.setValueFactory(doubleSpinnerValueFactory.get());
      orientationWeightYaw.setValueFactory(doubleSpinnerValueFactory.get());
      orientationWeightPitch.setValueFactory(doubleSpinnerValueFactory.get());
      orientationWeightRoll.setValueFactory(doubleSpinnerValueFactory.get());
      costPerStep.setValueFactory(doubleSpinnerValueFactory.get());
      footholdAreaWeight.setValueFactory(doubleSpinnerValueFactory.get());
      astarHeuristicsWeight.setValueFactory(doubleSpinnerValueFactory.get());

      ValkyrieAStarFootstepPlannerParameters parameters = planner.getParameters();
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
   }

   public void setSpinnersFromPlannerParameters()
   {
      ValkyrieAStarFootstepPlannerParameters parameters = planner.getParameters();
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
      bodyBoxOffsetX.getValueFactory().setValue(parameters.getBodyBoxOffset().getX());
      bodyBoxOffsetY.getValueFactory().setValue(parameters.getBodyBoxOffset().getY());
      bodyBoxOffsetZ.getValueFactory().setValue(parameters.getBodyBoxOffset().getZ());
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
   }
}
