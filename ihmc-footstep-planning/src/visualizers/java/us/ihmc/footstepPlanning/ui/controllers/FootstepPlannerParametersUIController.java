package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.shape.Rectangle;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty();
   private FootstepPlannerParametersBasics planningParameters;

   @FXML
   private CheckBox returnBestEffortPlan;
   @FXML
   private CheckBox performHeuristicSearchPolicies;

   @FXML
   private Spinner<Double> maxStepLength;
   @FXML
   private Spinner<Double> maxStepWidth;
   @FXML
   private Spinner<Double> minStepWidth;

   @FXML
   private Spinner<Double> minStepLength;
   @FXML
   private Spinner<Double> maxStepZ;
   @FXML
   private Spinner<Double> minSurfaceIncline;

   @FXML
   private Spinner<Double> maxStepYaw;
   @FXML
   private Spinner<Double> minStepYaw;
   @FXML
   private Spinner<Double> minFootholdPercent;

   @FXML
   private Rectangle stepShape;
   @FXML
   private Rectangle stanceFootShape;
   @FXML
   private Rectangle swingFootShape;
   @FXML
   private Rectangle clearanceBox;

   @FXML
   private Spinner<Double> minXClearance;
   @FXML
   private Spinner<Double> minYClearance;
   @FXML
   private Spinner<Double> cliffHeightSpinner;
   @FXML
   private Spinner<Double> cliffClearance;
   @FXML
   private Spinner<Double> maxXYWiggleSpinner;
   @FXML
   private Spinner<Double> maxYawWiggleSpinner;
   @FXML
   private Spinner<Double> wiggleInsideDeltaSpinner;

   @FXML
   private Spinner<Double> maxStepUpX;
   @FXML
   private Spinner<Double> stepUpHeight;
   @FXML
   private Spinner<Double> maxStepDownX;
   @FXML
   private Spinner<Double> stepDownHeight;

   private static final double footWidth = 0.15;
   private static final double footLength = 0.25;
   private static final double leftFootOriginX = 30;
   private static final double leftFootOriginY = 100;

   private static final double metersToPixel = 200;


   public FootstepPlannerParametersUIController()
   {
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlannerParametersBasics parameters)
   {
      parametersProperty.setPlannerParameters(parameters);
      this.planningParameters = parameters;
   }

   public void setupControls()
   {
      maxStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.7, 0.0, 0.05));
      maxStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.02));
      minStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.01));

      minStepLength.setValueFactory(new DoubleSpinnerValueFactory(-0.6, 0.0, 0.0, 0.05));
      maxStepZ.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.02));
      minSurfaceIncline.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.1));
      maxStepUpX.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.01));
      stepUpHeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.01));
      maxStepDownX.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.01));
      stepDownHeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.01));

      maxStepYaw.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.1));
      minStepYaw.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.1));
      minFootholdPercent.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      minXClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.01));
      minYClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.01));
      maxXYWiggleSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.005));
      maxYawWiggleSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.005));
      wiggleInsideDeltaSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.005));

      maxStepLength.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      minStepLength.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      maxStepWidth.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      minStepWidth.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      maxStepYaw.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      minStepYaw.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      minXClearance.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      minYClearance.getValueFactory().valueProperty().addListener(observable -> updateStepShape());
      
      cliffHeightSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      cliffClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
   }

   public void bindControls()
   {
      setupControls();

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParametersTopic, parameters -> planningParameters.set(parameters));

      parametersProperty.bidirectionalBindReturnBestEffortPlan(returnBestEffortPlan.selectedProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindPerformHeuristicSearchPolicies(performHeuristicSearchPolicies.selectedProperty(), observable -> publishParameters());

      parametersProperty.bidirectionalBindMaxStepReach(maxStepLength.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMaxStepWidth(maxStepWidth.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinStepWidth(minStepWidth.getValueFactory().valueProperty(), observable -> publishParameters());

      parametersProperty.bidirectionalBindMinStepLength(minStepLength.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMaxStepZ(maxStepZ.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinSurfaceIncline(minSurfaceIncline.getValueFactory().valueProperty(), observable -> publishParameters());

      parametersProperty.bidirectionalBindMaxStepYaw(maxStepYaw.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinStepYaw(minStepYaw.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinFootholdPercent(minFootholdPercent.getValueFactory().valueProperty(), observable -> publishParameters());

      parametersProperty.bidirectionalBindMinXClearanceFromStance(minXClearance.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinYClearanceFromStance(minYClearance.getValueFactory().valueProperty(), observable -> publishParameters());
      
      parametersProperty.bidirectionalBindCliffHeight(cliffHeightSpinner.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindCliffClearance(cliffClearance.getValueFactory().valueProperty(), observable -> publishParameters());

      parametersProperty.bidirectionalBindMaxWiggleXY(maxXYWiggleSpinner.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMaxWiggleYaw(maxYawWiggleSpinner.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindWiggleInsideDelta(wiggleInsideDeltaSpinner.getValueFactory().valueProperty(), observable -> publishParameters());

      parametersProperty.bidirectionalBindMaxXForStepUp(maxStepUpX.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinZToConsiderStepUp(stepUpHeight.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMaxXForStepDown(maxStepDownX.getValueFactory().valueProperty(), observable -> publishParameters());
      parametersProperty.bidirectionalBindMinZToConsiderStepDown(stepDownHeight.getValueFactory().valueProperty(), observable -> publishParameters());

      // these dimensions work best for valkyrie
      stanceFootShape.setHeight(footLength * metersToPixel);
      stanceFootShape.setWidth(footWidth * metersToPixel);
      stanceFootShape.setLayoutX(leftFootOriginX);
      stanceFootShape.setLayoutY(leftFootOriginY);

      swingFootShape.setHeight(footLength * metersToPixel);
      swingFootShape.setWidth(footWidth * metersToPixel);
      swingFootShape.setLayoutY(leftFootOriginY);

      updateStepShape();
   }

   private void publishParameters()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, planningParameters);
   }


   @FXML
   public void saveToFile()
   {
      planningParameters.getStoredPropertySet().save();
   }

   private void updateStepShape()
   {
      double worstYaw = maxStepYaw.getValue() > Math.abs(minStepYaw.getValue()) ? maxStepYaw.getValue() : minStepYaw.getValue();
      setStepShape(maxStepLength.getValue(), minStepLength.getValue(), maxStepWidth.getValue(), minStepWidth.getValue(), worstYaw);
   }

   private void setStepShape(double maxLength, double minLength, double maxWidth, double minWidth, double yaw)
   {
      double footCenterX = leftFootOriginX + 0.5 * footWidth * metersToPixel;
      double footCenterY = leftFootOriginY + 0.5 * footLength * metersToPixel;

      double furthestIn = Math.max(minWidth, minYClearance.getValue()) * metersToPixel;

      double width = maxWidth - minWidth;
      double height = maxLength - minLength;
      double xCenterInPanel = footCenterX + metersToPixel * minWidth;
      double yCenterInPanel = footCenterY - metersToPixel * maxLength;


      stepShape.setLayoutX(xCenterInPanel);
      stepShape.setWidth(metersToPixel * width);
      stepShape.setLayoutY(yCenterInPanel);
      stepShape.setHeight(metersToPixel * height);

      swingFootShape.setLayoutX(footCenterX + furthestIn - 0.5 * footWidth * metersToPixel);
      swingFootShape.setRotate(Math.toDegrees(yaw));

      clearanceBox.setLayoutX(leftFootOriginX + (0.5 * footWidth - minYClearance.getValue()) * metersToPixel);
      clearanceBox.setLayoutY(leftFootOriginY + (0.5 * footLength - minXClearance.getValue()) * metersToPixel);
      clearanceBox.setWidth(metersToPixel * (minYClearance.getValue() * 2.0));
      clearanceBox.setHeight(metersToPixel * (minXClearance.getValue() * 2.0));
   }
}
