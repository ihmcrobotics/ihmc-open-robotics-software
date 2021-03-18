package us.ihmc.quadrupedUI.uiControllers;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
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
   private Spinner<Double> endPhaseShift;
   @FXML
   private ComboBox<QuadrupedSpeed> quadrupedSpeed;

   @FXML
   private Spinner<Double> ambleSlowStepDuration;
   @FXML
   private Spinner<Double> ambleSlowEndDoubleSupportDuration;
   @FXML
   private Spinner<Double> ambleSlowMaxSpeed;

   @FXML
   private Spinner<Double> ambleMediumStepDuration;
   @FXML
   private Spinner<Double> ambleMediumEndDoubleSupportDuration;
   @FXML
   private Spinner<Double> ambleMediumMaxSpeed;

   @FXML
   private Spinner<Double> ambleFastStepDuration;
   @FXML
   private Spinner<Double> ambleFastEndDoubleSupportDuration;
   @FXML
   private Spinner<Double> ambleFastMaxSpeed;

   @FXML
   private Spinner<Double> trotSlowStepDuration;
   @FXML
   private Spinner<Double> trotSlowEndDoubleSupportDuration;
   @FXML
   private Spinner<Double> trotSlowMaxSpeed;

   @FXML
   private Spinner<Double> trotMediumStepDuration;
   @FXML
   private Spinner<Double> trotMediumEndDoubleSupportDuration;
   @FXML
   private Spinner<Double> trotMediumMaxSpeed;

   @FXML
   private Spinner<Double> trotFastStepDuration;
   @FXML
   private Spinner<Double> trotFastEndDoubleSupportDuration;
   @FXML
   private Spinner<Double> trotFastMaxSpeed;





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
      endPhaseShift.setValueFactory(new DoubleSpinnerValueFactory(0.0, 180, 0.0, 45));

      ambleSlowStepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      ambleSlowEndDoubleSupportDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 0.0, 0.05));
      ambleSlowMaxSpeed.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      ambleMediumStepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      ambleMediumEndDoubleSupportDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 0.0, 0.05));
      ambleMediumMaxSpeed.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      ambleFastStepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      ambleFastEndDoubleSupportDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 0.0, 0.05));
      ambleFastMaxSpeed.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      trotSlowStepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      trotSlowEndDoubleSupportDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 0.0, 0.05));
      trotSlowMaxSpeed.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      trotMediumStepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      trotMediumEndDoubleSupportDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 0.0, 0.05));
      trotMediumMaxSpeed.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      trotFastStepDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 5.0, 0.0, 0.05));
      trotFastEndDoubleSupportDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0, 10.0, 0.0, 0.05));
      trotFastMaxSpeed.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, 0.0, 0.05));

      ObservableList<QuadrupedSpeed> speedOptions = FXCollections.observableArrayList(QuadrupedSpeed.values);
      quadrupedSpeed.setItems(speedOptions);
      quadrupedSpeed.setValue(QuadrupedSpeed.SLOW);
   }

   public void bindControls()
   {
      setupControls();

      parametersProperty.bidirectionalBindStanceLength(stanceLength.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStanceWidth(stanceWidth.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindStepGroundClearance(stepGroundClearance.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindEndPhaseShift(endPhaseShift.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindAmbleSlowStepDuration(ambleSlowStepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindAmbleSlowEndDoubleSupportDuration(ambleSlowEndDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindAmbleSlowMaxSpeed(ambleSlowMaxSpeed.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindAmbleMediumStepDuration(ambleMediumStepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindAmbleMediumEndDoubleSupportDuration(ambleMediumEndDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindAmbleMediumMaxSpeed(ambleMediumMaxSpeed.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindAmbleFastStepDuration(ambleFastStepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindAmbleFastEndDoubleSupportDuration(ambleFastEndDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindAmbleFastMaxSpeed(ambleFastMaxSpeed.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindTrotSlowStepDuration(trotSlowStepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindTrotSlowEndDoubleSupportDuration(trotSlowEndDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindTrotSlowMaxSpeed(trotSlowMaxSpeed.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindTrotMediumStepDuration(trotMediumStepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindTrotMediumEndDoubleSupportDuration(trotMediumEndDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindTrotMediumMaxSpeed(trotMediumMaxSpeed.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindTrotFastStepDuration(trotFastStepDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindTrotFastEndDoubleSupportDuration(trotFastEndDoubleSupportDuration.getValueFactory().valueProperty());
      parametersProperty.bidirectionalBindTrotFastMaxSpeed(trotFastMaxSpeed.getValueFactory().valueProperty());

      parametersProperty.bidirectionalBindQuadrupedSpeed(quadrupedSpeed.valueProperty());

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
