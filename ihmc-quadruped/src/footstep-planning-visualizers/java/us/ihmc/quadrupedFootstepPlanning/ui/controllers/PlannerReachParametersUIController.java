package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import java.io.File;
import java.io.IOException;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.commons.PrintTools;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParameterKeys;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersBasics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

public class PlannerReachParametersUIController
{
   private JavaFXMessager messager;
   private PawStepPlannerParametersBasics planningParameters;


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

   private Topic<PawStepPlannerParametersReadOnly> plannerParametersTopic;

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

      JavaFXStoredPropertyMap javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(planningParameters);

      javaFXStoredPropertyMap.put(maxFrontStepReach, PawStepPlannerParameterKeys.maximumFrontStepReach);
      javaFXStoredPropertyMap.put(maxFrontStepLength, PawStepPlannerParameterKeys.maximumFrontStepLength);
      javaFXStoredPropertyMap.put(minFrontStepLength, PawStepPlannerParameterKeys.minimumFrontStepLength);
      javaFXStoredPropertyMap.put(maxHindStepReach, PawStepPlannerParameterKeys.maximumHindStepReach);
      javaFXStoredPropertyMap.put(maxHindStepLength, PawStepPlannerParameterKeys.maximumHindStepLength);
      javaFXStoredPropertyMap.put(minHindStepLength, PawStepPlannerParameterKeys.minimumHindStepLength);
      javaFXStoredPropertyMap.put(maxStepWidth, PawStepPlannerParameterKeys.maximumStepOutward);
      javaFXStoredPropertyMap.put(minStepWidth, PawStepPlannerParameterKeys.maximumStepInward);

      javaFXStoredPropertyMap.put(maxFrontStepLengthWhenSteppingUp, PawStepPlannerParameterKeys.maximumFrontStepLengthWhenSteppingUp);
      javaFXStoredPropertyMap.put(minFrontStepLengthWhenSteppingUp, PawStepPlannerParameterKeys.minimumFrontStepLengthWhenSteppingUp);
      javaFXStoredPropertyMap.put(maxHindStepLengthWhenSteppingUp, PawStepPlannerParameterKeys.maximumHindStepLengthWhenSteppingUp);
      javaFXStoredPropertyMap.put(minHindStepLengthWhenSteppingUp, PawStepPlannerParameterKeys.minimumHindStepLengthWhenSteppingUp);
      javaFXStoredPropertyMap.put(stepZForSteppingUp, PawStepPlannerParameterKeys.stepZForSteppingUp);

      javaFXStoredPropertyMap.put(maxFrontStepLengthWhenSteppingDown, PawStepPlannerParameterKeys.maximumFrontStepLengthWhenSteppingDown);
      javaFXStoredPropertyMap.put(minFrontStepLengthWhenSteppingDown, PawStepPlannerParameterKeys.minimumFrontStepLengthWhenSteppingDown);
      javaFXStoredPropertyMap.put(maxHindStepLengthWhenSteppingDown, PawStepPlannerParameterKeys.maximumHindStepLengthWhenSteppingDown);
      javaFXStoredPropertyMap.put(minHindStepLengthWhenSteppingDown, PawStepPlannerParameterKeys.minimumHindStepLengthWhenSteppingDown);
      javaFXStoredPropertyMap.put(stepZForSteppingDown, PawStepPlannerParameterKeys.stepZForSteppingDown);

      javaFXStoredPropertyMap.put(maxStepYaw, PawStepPlannerParameterKeys.maximumStepYawOutward);
      javaFXStoredPropertyMap.put(minStepYaw, PawStepPlannerParameterKeys.maximumStepYawInward);
      javaFXStoredPropertyMap.put(maxStepChangeZ, PawStepPlannerParameterKeys.maximumStepChangeZ);

      // set messager updates to update all stored properties and select JavaFX properties
      messager.addTopicListener(PawStepPlannerMessagerAPI.PlannerParametersTopic, parameters ->
      {
         planningParameters.set(parameters);

         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });

      // set JavaFX user input to update stored properties and publish messager message
      javaFXStoredPropertyMap.bindStoredToJavaFXUserInput();
      javaFXStoredPropertyMap.addAnyJavaFXValueChangedListener(() -> publishParameters());
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
}
