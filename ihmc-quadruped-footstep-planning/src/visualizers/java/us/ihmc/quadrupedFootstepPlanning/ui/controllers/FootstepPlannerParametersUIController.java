package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.quadrupedFootstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.quadrupedFootstepPlanning.ui.components.SettableFootstepPlannerParameters;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

import java.io.File;
import java.io.IOException;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty(this, "footstepPlannerParametersProperty");


   @FXML
   private Spinner<Double> maxFrontStepReach;
   @FXML
   private Spinner<Double> maxFrontStepLength;
   @FXML
   private Spinner<Double> minFrontStepLength;
   @FXML
   private Spinner<Double> maxHindStepReach;
   @FXML
   private Spinner<Double> maxHindStepLength;
   @FXML
   private Spinner<Double> minHindStepLength;
   @FXML
   private Spinner<Double> maxStepWidth;
   @FXML
   private Spinner<Double> minStepWidth;

   @FXML
   private Spinner<Double> maxFrontStepLengthWhenSteppingUp;
   @FXML
   private Spinner<Double> minFrontStepLengthWhenSteppingUp;
   @FXML
   private Spinner<Double> maxHindStepLengthWhenSteppingUp;
   @FXML
   private Spinner<Double> minHindStepLengthWhenSteppingUp;
   @FXML
   private Spinner<Double> stepZForSteppingUp;

   @FXML
   private Spinner<Double> maxFrontStepLengthWhenSteppingDown;
   @FXML
   private Spinner<Double> minFrontStepLengthWhenSteppingDown;
   @FXML
   private Spinner<Double> maxHindStepLengthWhenSteppingDown;
   @FXML
   private Spinner<Double> minHindStepLengthWhenSteppingDown;
   @FXML
   private Spinner<Double> stepZForSteppingDown;

   @FXML
   private Spinner<Double> maxStepYaw;
   @FXML
   private Spinner<Double> minStepYaw;
   @FXML
   private Spinner<Double> maxStepChangeZ;

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
   private Spinner<Double> projectInsideDistanceForExpansion;
   @FXML
   private Spinner<Double> projectInsideDistanceForPostProcessing;
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

   private Topic<FootstepPlannerParameters> plannerParametersTopic;

   public FootstepPlannerParametersUIController()
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

   public void setPlannerParametersTopic(Topic<FootstepPlannerParameters> plannerParametersTopic)
   {
      this.plannerParametersTopic = plannerParametersTopic;
   }


   public void setPlannerParameters(FootstepPlannerParameters parameters)
   {
      parametersProperty.setPlannerParameters(parameters);
   }

   public void setupControls()
   {
      maxFrontStepReach.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.7, 0.0, 0.05));
      maxFrontStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.0, 0.05));
      minFrontStepLength.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.05));
      maxHindStepReach.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.7, 0.0, 0.05));
      maxHindStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.0, 0.05));
      minHindStepLength.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.05));
      maxStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.02));
      minStepWidth.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.01));

      maxFrontStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.0, 0.05));
      minFrontStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.05));
      maxHindStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.0, 0.05));
      minHindStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.05));
      stepZForSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.05));

      maxFrontStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.0, 0.05));
      minFrontStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.05));
      maxHindStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.6, 0.0, 0.05));
      minHindStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.0, 0.0, 0.05));
      stepZForSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.05));


      maxStepYaw.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.05));
      minStepYaw.setValueFactory(new DoubleSpinnerValueFactory(-0.3, 0.0, 0.0, 0.05));
      maxStepChangeZ.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.05));

      maxWalkingSpeedMultiplier.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.01));

      bodyGroundClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.05));
      minXClearanceFromFoot.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.05));
      minYClearanceFromFoot.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.05));
      minSurfaceIncline.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.8, 0.0, 0.05));

      projectInsideDistanceForExpansion.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.1, 0.0, 0.01));
      projectInsideDistanceForPostProcessing.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.12, 0.0, 0.01));
      maximumXYWiggleDistance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.12, 0.0, 0.01));
      cliffHeightToAvoid.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.4, 0.0, 0.05));
      minFrontEndForwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.01));
      minFrontEndBackwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.01));
      minHindEndForwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.3, 0.0, 0.01));
      minHindEndBackwardDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.01));
      minLateralDistanceFromCliffBottoms.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.01));

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

      parametersProperty.bidirectionalBindMaximumFrontStepReach(maxFrontStepReach.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumFrontStepLength(maxFrontStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumFrontStepLength(minFrontStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumHindStepReach(maxHindStepReach.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumHindStepLength(maxHindStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumHindStepLength(minHindStepLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumStepWidth(maxStepWidth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumStepWidth(minStepWidth.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaximumFrontStepLengthWhenSteppingUp(maxFrontStepLengthWhenSteppingUp.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumFrontStepLengthWhenSteppingUp(minFrontStepLengthWhenSteppingUp.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumHindStepLengthWhenSteppingUp(maxHindStepLengthWhenSteppingUp.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumHindStepLengthWhenSteppingUp(minHindStepLengthWhenSteppingUp.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStepZForSteppingUp(stepZForSteppingUp.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaximumFrontStepLengthWhenSteppingDown(maxFrontStepLengthWhenSteppingDown.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumFrontStepLengthWhenSteppingDown(minFrontStepLengthWhenSteppingDown.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumHindStepLengthWhenSteppingDown(maxHindStepLengthWhenSteppingDown.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumHindStepLengthWhenSteppingDown(minHindStepLengthWhenSteppingDown.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStepZForSteppingDown(stepZForSteppingDown.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaximumStepYaw(maxStepYaw.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumStepYaw(minStepYaw.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumStepChangeZ(maxStepChangeZ.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindMaxWalkingSpeedMultiplier(maxWalkingSpeedMultiplier.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindBodyGroundClearance(bodyGroundClearance.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinXClearanceFromFoot(minXClearanceFromFoot.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinYClearanceFromFoot(minYClearanceFromFoot.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumSurfaceInclineRadians(minSurfaceIncline.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindProjectInsideDistanceForExpansion(projectInsideDistanceForExpansion.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindProjectInsideDistanceForPostProcessing(projectInsideDistanceForPostProcessing.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMaximumXYWiggleDistance(maximumXYWiggleDistance.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindCliffHeightToAvoid(cliffHeightToAvoid.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinFrontEndForwardDistanceFromCliffBottoms(minFrontEndForwardDistanceFromCliffBottoms.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinFrontEndBackwardDistanceFromCliffBottoms(minFrontEndBackwardDistanceFromCliffBottoms.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinHindEndForwardDistanceFromCliffBottoms(minHindEndForwardDistanceFromCliffBottoms.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinHindEndBackwardDistanceFromCliffBottoms(minHindEndBackwardDistanceFromCliffBottoms.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinLateralDistanceFromCliffBottoms(minLateralDistanceFromCliffBottoms.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindDistanceWeight(distanceWeight.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindYawWeight(yawWeight.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindXGaitWeight(xGaitWeight.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindStepUpWeight(stepUpWeight.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStepDownWeight(stepDownWeight.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindCostPerStep(costPerStep.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindHeuristicsWeight(heuristicsWeight.getValueFactory().valueProperty());

      messager.bindBidirectional(plannerParametersTopic, parametersProperty, createConverter(), true);
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

      filePropertyHelper.saveProperty("maxFrontStepReach", maxFrontStepReach.getValue());
      filePropertyHelper.saveProperty("maxFrontStepLength", maxFrontStepLength.getValue());
      filePropertyHelper.saveProperty("minFrontStepLength", minFrontStepLength.getValue());
      filePropertyHelper.saveProperty("maxHindStepReach", maxHindStepReach.getValue());
      filePropertyHelper.saveProperty("maxHindStepLength", maxHindStepLength.getValue());
      filePropertyHelper.saveProperty("minHindStepLength", minHindStepLength.getValue());
      filePropertyHelper.saveProperty("maxStepWidth", maxStepWidth.getValue());
      filePropertyHelper.saveProperty("minStepWidth", minStepWidth.getValue());

      filePropertyHelper.saveProperty("maxFrontStepLengthWhenSteppingUp", maxFrontStepLengthWhenSteppingUp.getValue());
      filePropertyHelper.saveProperty("minFrontStepLengthWhenSteppingUp", minFrontStepLengthWhenSteppingUp.getValue());
      filePropertyHelper.saveProperty("maxHindStepLengthWhenSteppingUp", maxHindStepLengthWhenSteppingUp.getValue());
      filePropertyHelper.saveProperty("minHindStepLengthWhenSteppingUp", minHindStepLengthWhenSteppingUp.getValue());
      filePropertyHelper.saveProperty("stepZForSteppingUp", stepZForSteppingUp.getValue());

      filePropertyHelper.saveProperty("maxFrontStepLengthWhenSteppingDown", maxFrontStepLengthWhenSteppingDown.getValue());
      filePropertyHelper.saveProperty("minFrontStepLengthWhenSteppingDown", minFrontStepLengthWhenSteppingDown.getValue());
      filePropertyHelper.saveProperty("maxHindStepLengthWhenSteppingDown", maxHindStepLengthWhenSteppingDown.getValue());
      filePropertyHelper.saveProperty("minHindStepLengthWhenSteppingDown", minHindStepLengthWhenSteppingDown.getValue());
      filePropertyHelper.saveProperty("stepZForSteppingDown", stepZForSteppingDown.getValue());

      filePropertyHelper.saveProperty("maxStepYaw", maxStepYaw.getValue());
      filePropertyHelper.saveProperty("minStepYaw", minStepYaw.getValue());
      filePropertyHelper.saveProperty("maxStepChangeZ", maxStepChangeZ.getValue());

      filePropertyHelper.saveProperty("maxWalkingSpeedMultiplier", maxWalkingSpeedMultiplier.getValue());

      filePropertyHelper.saveProperty("bodyGroundClearance", bodyGroundClearance.getValue());
      filePropertyHelper.saveProperty("minXClearanceFromFoot", minXClearanceFromFoot.getValue());
      filePropertyHelper.saveProperty("minYClearanceFromFoot", minYClearanceFromFoot.getValue());
      filePropertyHelper.saveProperty("minSurfaceIncline", minSurfaceIncline.getValue());

      filePropertyHelper.saveProperty("projectInsideDistanceForExpansion", projectInsideDistanceForExpansion.getValue());
      filePropertyHelper.saveProperty("projectInsideDistanceForPostProcessing", projectInsideDistanceForPostProcessing.getValue());
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
      if ((value = filePropertyHelper.loadDoubleProperty("maxFrontStepReach")) != null)
         maxFrontStepReach.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxFrontStepLength")) != null)
         maxFrontStepLength.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minFrontStepLength")) != null)
         minFrontStepLength.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxHindStepReach")) != null)
         maxHindStepReach.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxHindStepLength")) != null)
         maxHindStepLength.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minHindStepLength")) != null)
         minHindStepLength.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxStepWidth")) != null)
         maxStepWidth.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minStepWidth")) != null)
         minStepWidth.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("maxFrontStepLengthWhenSteppingUp")) != null)
         maxFrontStepLengthWhenSteppingUp.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minFrontStepLengthWhenSteppingUp")) != null)
         minFrontStepLengthWhenSteppingUp.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxHindStepLengthWhenSteppingUp")) != null)
         maxHindStepLengthWhenSteppingUp.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minHindStepLengthWhenSteppingUp")) != null)
         minHindStepLengthWhenSteppingUp.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("stepZForSteppingUp")) != null)
         stepZForSteppingUp.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("maxFrontStepLengthWhenSteppingDown")) != null)
         maxFrontStepLengthWhenSteppingDown.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minFrontStepLengthWhenSteppingDown")) != null)
         minFrontStepLengthWhenSteppingDown.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxHindStepLengthWhenSteppingDown")) != null)
         maxHindStepLengthWhenSteppingDown.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minHindStepLengthWhenSteppingDown")) != null)
         minHindStepLengthWhenSteppingDown.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("stepZForSteppingDown")) != null)
         stepZForSteppingDown.getValueFactory().setValue(value);

      if ((value = filePropertyHelper.loadDoubleProperty("maxStepYaw")) != null)
         maxStepYaw.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("minStepYaw")) != null)
         minStepYaw.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("maxStepChangeZ")) != null)
         maxStepChangeZ.getValueFactory().setValue(value);

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

      if ((value = filePropertyHelper.loadDoubleProperty("projectInsideDistanceForExpansion")) != null)
         projectInsideDistanceForExpansion.getValueFactory().setValue(value);
      if ((value = filePropertyHelper.loadDoubleProperty("projectInsideDistanceForPostProcessing")) != null)
         projectInsideDistanceForPostProcessing.getValueFactory().setValue(value);
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
