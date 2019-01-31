package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import javafx.scene.shape.Rectangle;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.footstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty(this, "footstepPlannerParametersProperty");

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

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParameters(FootstepPlannerParameters parameters)
   {
      parametersProperty.setPlannerParameters(parameters);
   }

   public void setupControls()
   {
      maxStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.7, 0.0, 0.05));
      maxStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.02));
      minStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.01));

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

      maxStepLength.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      minStepLength.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      maxStepWidth.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      minStepWidth.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      maxStepYaw.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      minStepYaw.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      minXClearance.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      minYClearance.getValueFactory().valueProperty().addListener((ChangeListener) -> updateStepShape());
      
      cliffHeightSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      cliffClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
   }

   public void bindControls()
   {
      setupControls();

      parametersProperty.bidirectionalBindReturnBestEffortPlan(returnBestEffortPlan.selectedProperty());
      parametersProperty.bidirectionalBindPerformHeuristicSearchPolicies(performHeuristicSearchPolicies.selectedProperty());

      parametersProperty.bidirectionalBindMaxStepReach(maxStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaxStepWidth(maxStepWidth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinStepWidth(minStepWidth.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMinStepLength(minStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaxStepZ(maxStepZ.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinSurfaceIncline(minSurfaceIncline.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaxStepYaw(maxStepYaw.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinStepYaw(minStepYaw.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinFootholdPercent(minFootholdPercent.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMinXClearanceFromStance(minXClearance.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinYClearanceFromStance(minYClearance.getValueFactory().valueProperty());
      
      parametersProperty.bidirectionalBindCliffHeight(cliffHeightSpinner.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindCliffClearance(cliffClearance.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaxWiggleXY(maxXYWiggleSpinner.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaxWiggleYaw(maxYawWiggleSpinner.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindWiggleInsideDelta(wiggleInsideDeltaSpinner.getValueFactory().valueProperty());

      parametersProperty.bidirectionBindMaxXForStepUp(maxStepUpX.getValueFactory().valueProperty());
      parametersProperty.bidirectionBindMinZToConsiderStepUp(stepUpHeight.getValueFactory().valueProperty());
      parametersProperty.bidirectionBindMaxXForStepDown(maxStepDownX.getValueFactory().valueProperty());
      parametersProperty.bidirectionBindMinZToConsiderStepDown(stepDownHeight.getValueFactory().valueProperty());

      messager.bindBidirectional(FootstepPlannerMessagerAPI.PlannerParametersTopic, parametersProperty, createConverter(), true);

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

   private PropertyToMessageTypeConverter<FootstepPlannerParameters, SettableFootstepPlannerParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<FootstepPlannerParameters, SettableFootstepPlannerParameters>()
      {
         @Override
         public FootstepPlannerParameters convert(SettableFootstepPlannerParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettableFootstepPlannerParameters interpret(FootstepPlannerParameters messageContent)
         {
            return new SettableFootstepPlannerParameters(messageContent);
         }
      };
   }
}
