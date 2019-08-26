package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParameterKeys;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.ui.properties.JavaFXStoredPropertyMap;

import java.io.File;
import java.io.IOException;

public class PawStepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private PawStepPlannerParametersBasics planningParameters;

   @FXML
   private Spinner<Double> maxWalkingSpeedMultiplier;

   @FXML
   private Spinner<Double> bodyGroundClearance;
   @FXML
   private Spinner<Double> minXClearanceFromFoot;
   @FXML
   private Spinner<Double> minYClearanceFromFoot;
   @FXML
   private Spinner<Double> minSurfaceIncline;

   @FXML
   private Spinner<Double> projectInsideDistance;
   @FXML
   private Spinner<Double> maximumXYWiggleDistance;
   @FXML
   private Spinner<Double> cliffHeightToAvoid;
   @FXML
   private Spinner<Double> minFrontEndForwardDistanceFromCliffBottoms;
   @FXML
   private Spinner<Double> minFrontEndBackwardDistanceFromCliffBottoms;
   @FXML
   private Spinner<Double> minHindEndForwardDistanceFromCliffBottoms;
   @FXML
   private Spinner<Double> minHindEndBackwardDistanceFromCliffBottoms;
   @FXML
   private Spinner<Double> minLateralDistanceFromCliffBottoms;


   @FXML
   private Spinner<Double> distanceWeight;
   @FXML
   private Spinner<Double> yawWeight;
   @FXML
   private Spinner<Double> xGaitWeight;

   @FXML
   private Spinner<Double> costPerStep;
   @FXML
   private Spinner<Double> stepUpWeight;
   @FXML
   private Spinner<Double> stepDownWeight;
   @FXML
   private Spinner<Double> heuristicsWeight;


   private static final String CONFIGURATION_FILE_NAME = "./Configurations/footstepPlannerParameters.txt";
   private FilePropertyHelper filePropertyHelper;

   private Topic<PawStepPlannerParametersReadOnly> plannerParametersTopic;

   public PawStepPlannerParametersUIController()
   {
      File configurationFile = new File(CONFIGURATION_FILE_NAME);
      try
      {
         configurationFile.getParentFile().mkdirs();
         configurationFile.createNewFile();
         filePropertyHelper = new FilePropertyHelper(configurationFile);
      }
      catch (IOException e)
      {
         System.out.println(configurationFile.getAbsolutePath());
         e.printStackTrace();
      }
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPlannerParametersTopic(Topic<PawStepPlannerParametersReadOnly> plannerParametersTopic)
   {
      this.plannerParametersTopic = plannerParametersTopic;
   }


   public void setPlannerParameters(PawStepPlannerParametersBasics parameters)
   {
      planningParameters = parameters;
   }

   public void setupControls()
   {
      maxWalkingSpeedMultiplier.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.01));

      bodyGroundClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.05));
      minXClearanceFromFoot.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.05));
      minYClearanceFromFoot.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.05));
      minSurfaceIncline.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.8, 0.0, 0.05));

      projectInsideDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      maximumXYWiggleDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.01));
      cliffHeightToAvoid.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
      minFrontEndForwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      minFrontEndBackwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      minHindEndForwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      minHindEndBackwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));
      minLateralDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.01));

      distanceWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
      yawWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
      xGaitWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));

      costPerStep.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
      stepUpWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
      stepDownWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
      heuristicsWeight.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.1));
   }

   public void bindControls()
   {
      setupControls();

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);

      javaFXStoredPropertyMap.put(maxWalkingSpeedMultiplier, PawStepPlannerParameterKeys.maxWalkingSpeedMultiplier);

      javaFXStoredPropertyMap.put(bodyGroundClearance, PawStepPlannerParameterKeys.bodyGroundClearance);
      javaFXStoredPropertyMap.put(minXClearanceFromFoot, PawStepPlannerParameterKeys.minXClearanceFromPaw);
      javaFXStoredPropertyMap.put(minYClearanceFromFoot, PawStepPlannerParameterKeys.minYClearanceFromPaw);
      javaFXStoredPropertyMap.put(minSurfaceIncline, PawStepPlannerParameterKeys.minimumSurfaceInclineRadians);

      javaFXStoredPropertyMap.put(projectInsideDistance, PawStepPlannerParameterKeys.projectInsideDistance);
      javaFXStoredPropertyMap.put(maximumXYWiggleDistance, PawStepPlannerParameterKeys.maximumXYWiggleDistance);

      javaFXStoredPropertyMap.put(cliffHeightToAvoid, PawStepPlannerParameterKeys.cliffHeightToAvoid);
      javaFXStoredPropertyMap.put(minFrontEndForwardDistanceFromCliffBottoms, PawStepPlannerParameterKeys.minimumFrontEndForwardDistanceFromCliffBottoms);
      javaFXStoredPropertyMap.put(minFrontEndBackwardDistanceFromCliffBottoms, PawStepPlannerParameterKeys.minimumFrontEndBackwardDistanceFromCliffBottoms);
      javaFXStoredPropertyMap.put(minHindEndForwardDistanceFromCliffBottoms, PawStepPlannerParameterKeys.minimumHindEndForwardDistanceFromCliffBottoms);
      javaFXStoredPropertyMap.put(minHindEndBackwardDistanceFromCliffBottoms, PawStepPlannerParameterKeys.minimumHindEndBackwardDistanceFromCliffBottoms);
      javaFXStoredPropertyMap.put(minLateralDistanceFromCliffBottoms, PawStepPlannerParameterKeys.minimumLateralDistanceFromCliffBottoms);

      javaFXStoredPropertyMap.put(distanceWeight, PawStepPlannerParameterKeys.distanceWeight);
      javaFXStoredPropertyMap.put(yawWeight, PawStepPlannerParameterKeys.yawWeight);
      javaFXStoredPropertyMap.put(xGaitWeight, PawStepPlannerParameterKeys.xGaitWeight);

      javaFXStoredPropertyMap.put(stepUpWeight, PawStepPlannerParameterKeys.stepUpWeight);
      javaFXStoredPropertyMap.put(stepDownWeight, PawStepPlannerParameterKeys.stepDownWeight);
      javaFXStoredPropertyMap.put(costPerStep, PawStepPlannerParameterKeys.costPerStep);
      javaFXStoredPropertyMap.put(heuristicsWeight, PawStepPlannerParameterKeys.heuristicsInflationWeight);


      // set messager updates to update all stored properties and select JavaFX properties
      messager.registerTopicListener(PawStepPlannerMessagerAPI.PlannerParametersTopic, parameters ->
      {
         planningParameters.set(parameters);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      // set JavaFX user input to update stored properties and publish messager message
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.bindToJavaFXUserInput(() -> publishParameters());
   }

   private void publishParameters()
   {
      messager.submitMessage(PawStepPlannerMessagerAPI.PlannerParametersTopic, planningParameters);
   }

   @FXML
   public void saveToFile()
   {
      if (filePropertyHelper == null)
      {
         PrintTools.warn("Can not save to file.");
         return;
      }

      System.out.println("Saving Parameters to file.");

      filePropertyHelper.saveProperty("maxWalkingSpeedMultiplier", maxWalkingSpeedMultiplier.getValue());

      filePropertyHelper.saveProperty("bodyGroundClearance", bodyGroundClearance.getValue());
      filePropertyHelper.saveProperty("minXClearanceFromFoot", minXClearanceFromFoot.getValue());
      filePropertyHelper.saveProperty("minYClearanceFromFoot", minYClearanceFromFoot.getValue());
      filePropertyHelper.saveProperty("minSurfaceIncline", minSurfaceIncline.getValue());

      filePropertyHelper.saveProperty("projectInsideDistance", projectInsideDistance.getValue());
      filePropertyHelper.saveProperty("cliffHeightToAvoid", cliffHeightToAvoid.getValue());
      filePropertyHelper.saveProperty("minFrontEndForwardDistanceFromCliffBottoms", minFrontEndForwardDistanceFromCliffBottoms.getValue());
      filePropertyHelper.saveProperty("minFrontEndBackwardDistanceFromCliffBottoms", minFrontEndBackwardDistanceFromCliffBottoms.getValue());
      filePropertyHelper.saveProperty("minHindEndForwardDistanceFromCliffBottoms", minHindEndForwardDistanceFromCliffBottoms.getValue());
      filePropertyHelper.saveProperty("minHindEndBackwardDistanceFromCliffBottoms", minHindEndBackwardDistanceFromCliffBottoms.getValue());
      filePropertyHelper.saveProperty("minLateralDistanceFromCliffBottoms", minLateralDistanceFromCliffBottoms.getValue());

      filePropertyHelper.saveProperty("distanceWeight", distanceWeight.getValue());
      filePropertyHelper.saveProperty("yawWeight", yawWeight.getValue());
      filePropertyHelper.saveProperty("xGaitWeight", xGaitWeight.getValue());

      filePropertyHelper.saveProperty("costPerStep", costPerStep.getValue());
      filePropertyHelper.saveProperty("stepUpWeight", stepUpWeight.getValue());
      filePropertyHelper.saveProperty("stepDownWeight", stepDownWeight.getValue());
      filePropertyHelper.saveProperty("heuristicsWeight", heuristicsWeight.getValue());
   }

   public void loadFromFile()
   {
      if (filePropertyHelper == null)
      {
         return;
      }

      Double value;

      if ((value = filePropertyHelper.loadDoubleProperty("maxWalkingSpeedMultiplier")) != null)
         maxWalkingSpeedMultiplier.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("bodyGroundClearance")) != null)
         bodyGroundClearance.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minXClearanceFromFoot")) != null)
         minXClearanceFromFoot.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minYClearanceFromFoot")) != null)
         minYClearanceFromFoot.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minSurfaceIncline")) != null)
         minSurfaceIncline.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("projectInsideDistance")) != null)
         projectInsideDistance.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("cliffHeightToAvoid")) != null)
         cliffHeightToAvoid.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minFrontEndForwardDistanceFromCliffBottoms")) != null)
         minFrontEndForwardDistanceFromCliffBottoms.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minFrontEndBackwardDistanceFromCliffBottoms")) != null)
         minFrontEndBackwardDistanceFromCliffBottoms.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minHindEndForwardDistanceFromCliffBottoms")) != null)
         minHindEndForwardDistanceFromCliffBottoms.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minHindEndBackwardDistanceFromCliffBottoms")) != null)
         minHindEndBackwardDistanceFromCliffBottoms.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minLateralDistanceFromCliffBottoms")) != null)
         minLateralDistanceFromCliffBottoms.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("distanceWeight")) != null)
         distanceWeight.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("xGaitWeight")) != null)
         xGaitWeight.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("yawWeight")) != null)
         yawWeight.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("costPerStep")) != null)
         costPerStep.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("stepUpWeight")) != null)
         stepUpWeight.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("stepDownWeight")) != null)
         stepDownWeight.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("heuristicsWeight")) != null)
         heuristicsWeight.getValueFactory().setValue(value);
   }
}
