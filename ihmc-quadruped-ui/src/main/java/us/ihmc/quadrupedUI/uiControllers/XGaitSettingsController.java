package us.ihmc.quadrupedUI.uiControllers;

import javafx.fxml.FXML;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedUI.QuadrupedUIMessagerAPI;
import us.ihmc.quadrupedUI.QuadrupedXGaitSettingsProperty;

public class XGaitSettingsController
{
   private JavaFXMessager messager;
   private QuadrupedXGaitSettingsProperty parametersProperty;

   @FXML
   private Spinner<Double> stanceLength;
   @FXML
   private Spinner<Double> stanceWidth;
   @FXML
   private Spinner<Double> stepGroundClearance;

   @FXML
   private Spinner<Double> stepDuration;
   @FXML
   private Spinner<Double> endDoubleSupportDuration;
   @FXML
   private Spinner<Double> endPhaseShift;



   public void attachMessager(JavaFXMessager messager, QuadrupedXGaitSettingsReadOnly defaultXGaitSettings)
   {
      this.messager = messager;
      parametersProperty = new QuadrupedXGaitSettingsProperty(this, "xGaitSettingsProperty", defaultXGaitSettings);
   }

   public void setXGaitSettings(QuadrupedXGaitSettings parameters)
   {
      parametersProperty.set(parameters);
   }

   private void setupControls()
   {
      stanceLength.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      stanceWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.75, 0.0, 0.02));
      stepGroundClearance.setValueFactory(new DoubleSpinnerValueFactory(0.0, 0.2, 0.0, 0.01));

      stepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      endPhaseShift.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.5, 0.0, 0.05));
      endPhaseShift.setValueFactory(new DoubleSpinnerValueFactory(0.0, 180, 0.0, 45));
   }

   public void bindControls()
   {
      setupControls();

      parametersProperty.bidirectionalBindStanceLength(stanceLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStanceWidth(stanceWidth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStepGroundClearance(stepGroundClearance.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindStepDuration(stepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindEndDoubleSupportDuration(endDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindEndPhaseShift(endPhaseShift.getValueFactory().valueProperty());

      messager.bindBidirectional(QuadrupedUIMessagerAPI.XGaitSettingsTopic, parametersProperty, createConverter(), true);
   }



   private PropertyToMessageTypeConverter<QuadrupedXGaitSettingsReadOnly, QuadrupedXGaitSettings> createConverter()
   {
      return new PropertyToMessageTypeConverter<QuadrupedXGaitSettingsReadOnly, QuadrupedXGaitSettings>()
      {
         @Override
         public QuadrupedXGaitSettingsReadOnly convert(QuadrupedXGaitSettings propertyValue)
         {
            return propertyValue;
         }

         @Override
         public QuadrupedXGaitSettings interpret(QuadrupedXGaitSettingsReadOnly messageContent)
         {
            return new QuadrupedXGaitSettings(messageContent);
         }
      };
   }
}
