package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.quadrupedFootstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.quadrupedFootstepPlanning.ui.components.SettablePawPlannerParameters;
import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

import java.io.File;
import java.io.IOException;

public class FootstepPlannerParametersUIController
{
   private JavaFXMessager messager;
   private final FootstepPlannerParametersProperty parametersProperty = new FootstepPlannerParametersProperty(this, "footstepPlannerParametersProperty");

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

   private Topic<PawPlannerParameters> plannerParametersTopic;

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

   public void setPlannerParametersTopic(Topic<PawPlannerParameters> plannerParametersTopic)
   {
      this.plannerParametersTopic = plannerParametersTopic;
   }


   public void setPlannerParameters(PawPlannerParameters parameters)
   {
      parametersProperty.setPlannerParameters(parameters);
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

      parametersProperty.bidirectionalBindMaxWalkingSpeedMultiplier(maxWalkingSpeedMultiplier.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindBodyGroundClearance(bodyGroundClearance.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinXClearanceFromFoot(minXClearanceFromFoot.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinYClearanceFromFoot(minYClearanceFromFoot.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindMinimumSurfaceInclineRadians(minSurfaceIncline.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindProjectInsideDistance(projectInsideDistance.getValueFactory().valueProperty());
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




   private PropertyToMessageTypeConverter<PawPlannerParameters, SettablePawPlannerParameters> createConverter()
   {
      return new PropertyToMessageTypeConverter<PawPlannerParameters, SettablePawPlannerParameters>()
      {
         @Override
         public PawPlannerParameters convert(SettablePawPlannerParameters propertyValue)
         {
            return propertyValue;
         }

         @Override
         public SettablePawPlannerParameters interpret(PawPlannerParameters messageContent)
         {
            return new SettablePawPlannerParameters(messageContent);
         }
      };
   }
}
