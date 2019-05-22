package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.ui.components.FootstepPlannerParametersProperty;
import us.ihmc.quadrupedFootstepPlanning.ui.components.SettableFootstepPlannerParameters;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

import java.io.File;
import java.io.IOException;

public class PlannerReachParametersUIController
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




   private static final String CONFIGURATION_FILE_NAME = "./Configurations/footstepPlannerParameters.txt";
   private FilePropertyHelper filePropertyHelper;

   private Topic<FootstepPlannerParameters> plannerParametersTopic;

   public PlannerReachParametersUIController()
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
      maxFrontStepReach.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      maxFrontStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      minFrontStepLength.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.05));
      maxHindStepReach.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      maxHindStepLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      minHindStepLength.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.05));
      maxStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.02));
      minStepWidth.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.01));

      maxFrontStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      minFrontStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.05));
      maxHindStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      minHindStepLengthWhenSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.05));
      stepZForSteppingUp.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.5, 0.0, 0.05));

      maxFrontStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      minFrontStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.05));
      maxHindStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.6, 0.0, 0.05));
      minHindStepLengthWhenSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-1.5, 0.0, 0.0, 0.05));
      stepZForSteppingDown.setValueFactory(new DoubleSpinnerValueFactory(-0.5, 0.5, 0.0, 0.05));


      maxStepYaw.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
      minStepYaw.setValueFactory(new DoubleSpinnerValueFactory(-1.0, 0.0, 0.0, 0.05));
      maxStepChangeZ.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));
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
