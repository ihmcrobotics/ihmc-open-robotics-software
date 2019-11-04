package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;

public class BodyCollisionCheckingUIController
{
   private JavaFXMessager messager;
   private FootstepPlannerParametersBasics planningParameters;

   @FXML
   private CheckBox enableBodyCollisionChecking;
   @FXML
   private CheckBox enablePathCollisionChecking;

   @FXML
   private Spinner<Double> bodyDepth;
   @FXML
   private Spinner<Double> bodyWidth;
   @FXML
   private Spinner<Double> bodyHeight;
   @FXML
   private Spinner<Integer> numberOfBoundingBoxChecks;

   @FXML
   private Spinner<Double> bodyBoxBaseX;
   @FXML
   private Spinner<Double> bodyBoxBaseY;
   @FXML
   private Spinner<Double> bodyBoxBaseZ;

   @FXML
   private Spinner<Double> maximum2dDistanceFromBoundingBoxToPenalize;
   @FXML
   private Spinner<Double> boundingBoxCost;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

   }

   public void setPlannerParameters(FootstepPlannerParametersBasics plannerParameters)
   {
      this.planningParameters = plannerParameters;
   }

   public void setupControls()
   {
      bodyWidth.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      bodyDepth.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
      bodyHeight.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 2.0, 0.0, 0.1));

      bodyBoxBaseX.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-1.0, 1.0, 0.0, 0.05));
      bodyBoxBaseY.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-1.0, 1.0, 0.0, 0.05));
      bodyBoxBaseZ.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.1));

      maximum2dDistanceFromBoundingBoxToPenalize.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
      boundingBoxCost.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(0.0, 10000.0, 0.0, 10.0));

      numberOfBoundingBoxChecks.setValueFactory(new IntegerSpinnerValueFactory(1, Integer.MAX_VALUE));
   }

   public void bindControls()
   {
      setupControls();

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);
      javaFXStoredPropertyMap.put(enableBodyCollisionChecking, FootstepPlannerParameterKeys.checkForBodyBoxCollisions);
      javaFXStoredPropertyMap.put(enablePathCollisionChecking, FootstepPlannerParameterKeys.checkForPathCollisions);
      javaFXStoredPropertyMap.put(numberOfBoundingBoxChecks, FootstepPlannerParameterKeys.numberOfBoundingBoxChecks);
      javaFXStoredPropertyMap.put(bodyDepth, FootstepPlannerParameterKeys.bodyBoxDepth);
      javaFXStoredPropertyMap.put(bodyHeight, FootstepPlannerParameterKeys.bodyBoxHeight);
      javaFXStoredPropertyMap.put(bodyWidth, FootstepPlannerParameterKeys.bodyBoxWidth);
      javaFXStoredPropertyMap.put(bodyBoxBaseX, FootstepPlannerParameterKeys.bodyBoxBaseX);
      javaFXStoredPropertyMap.put(bodyBoxBaseY, FootstepPlannerParameterKeys.bodyBoxBaseY);
      javaFXStoredPropertyMap.put(bodyBoxBaseZ, FootstepPlannerParameterKeys.bodyBoxBaseZ);
      javaFXStoredPropertyMap.put(boundingBoxCost, FootstepPlannerParameterKeys.boundingBoxCost);
      javaFXStoredPropertyMap.put(maximum2dDistanceFromBoundingBoxToPenalize, FootstepPlannerParameterKeys.maximum2dDistanceFromBoundingBoxToPenalize);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.PlannerParametersTopic, v ->
      {
         planningParameters.set(v);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(this::publishParameters);
   }

   private void publishParameters()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParametersTopic, planningParameters);
   }
}
