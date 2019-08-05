package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.shape.Rectangle;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
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

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters.getStoredPropertySet());
      javaFXStoredPropertyMap.put(returnBestEffortPlan, FootstepPlannerParameterKeys.returnBestEffortPlan);
      javaFXStoredPropertyMap.put(performHeuristicSearchPolicies, FootstepPlannerParameterKeys.performHeuristicSearchPolicies);
      javaFXStoredPropertyMap.put(maxStepLength, FootstepPlannerParameterKeys.maxStepReach);
      javaFXStoredPropertyMap.put(maxStepWidth, FootstepPlannerParameterKeys.maxStepWidth);
      javaFXStoredPropertyMap.put(minStepWidth, FootstepPlannerParameterKeys.minStepWidth);
      javaFXStoredPropertyMap.put(minStepLength, FootstepPlannerParameterKeys.maxStepWidth);
      javaFXStoredPropertyMap.put(maxStepZ, FootstepPlannerParameterKeys.minStepLength);
      javaFXStoredPropertyMap.put(minSurfaceIncline, FootstepPlannerParameterKeys.minSurfaceIncline);
      javaFXStoredPropertyMap.put(maxStepYaw, FootstepPlannerParameterKeys.maxStepYaw);
      javaFXStoredPropertyMap.put(minStepYaw, FootstepPlannerParameterKeys.minStepYaw);
      javaFXStoredPropertyMap.put(minFootholdPercent, FootstepPlannerParameterKeys.minFootholdPercent);
      javaFXStoredPropertyMap.put(minXClearance, FootstepPlannerParameterKeys.minXClearanceFromStance);
      javaFXStoredPropertyMap.put(minYClearance, FootstepPlannerParameterKeys.minYClearanceFromStance);
      javaFXStoredPropertyMap.put(cliffHeightSpinner, FootstepPlannerParameterKeys.cliffHeightToAvoid);
      javaFXStoredPropertyMap.put(cliffClearance, FootstepPlannerParameterKeys.minimumDistanceFromCliffBottoms);
      javaFXStoredPropertyMap.put(maxXYWiggleSpinner, FootstepPlannerParameterKeys.maximumXYWiggleDistance);
      javaFXStoredPropertyMap.put(maxYawWiggleSpinner, FootstepPlannerParameterKeys.maximumYawWiggle);
      javaFXStoredPropertyMap.put(wiggleInsideDeltaSpinner, FootstepPlannerParameterKeys.wiggleInsideDelta);
      javaFXStoredPropertyMap.put(maxStepUpX, FootstepPlannerParameterKeys.maximumStepReachWhenSteppingUp);
      javaFXStoredPropertyMap.put(stepUpHeight, FootstepPlannerParameterKeys.maximumStepZWhenSteppingUp);
      javaFXStoredPropertyMap.put(maxStepDownX, FootstepPlannerParameterKeys.maximumStepXWhenForwardAndDown);
      javaFXStoredPropertyMap.put(stepDownHeight, FootstepPlannerParameterKeys.maximumStepZWhenForwardAndDown);

      // set messager updates to update all stored properties and select JavaFX properties
      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParametersTopic, parameters ->
      {
         planningParameters.set(parameters);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      // set JavaFX user input to update stored properties and publish messager message
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(() -> publishParameters());

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
