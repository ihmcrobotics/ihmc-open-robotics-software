package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class StepGeneratorParametersPaneController
{
   @FXML
   private Slider trajectoryDurationSlider;

   @FXML
   private Slider swingHeightSlider;
   @FXML
   private Slider swingDurationSlider;
   @FXML
   private Slider transferDurationSlider;
   @FXML
   private Spinner<Double> maxStepLengthSpinner;
   @FXML
   private Spinner<Double> defaultStepWidthSpinner;
   @FXML
   private Spinner<Double> minStepWidthSpinner;
   @FXML
   private Spinner<Double> maxStepWidthSpinner;
   @FXML
   private Spinner<Double> turnStepWidth;
   @FXML
   private Spinner<Double> turnMaxAngleInwardSpinner;
   @FXML
   private Spinner<Double> turnMaxAngleOutwardSpinner;

   private final JoystickStepParametersProperty stepParametersProperty = new JoystickStepParametersProperty(this, "stepParameters");

   public StepGeneratorParametersPaneController()
   {
   }

   public void initialize(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters)
   {
      JoystickStepParameters initialParameters = new JoystickStepParameters();
      initialParameters.set(walkingControllerParameters);
      initialParameters.setSwingHeight(0.025);
      stepParametersProperty.set(initialParameters);

      swingHeightSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      maxStepLengthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getMaxStepLength(), 0.05));
      defaultStepWidthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getDefaultStepWidth(), 0.05));
      minStepWidthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getMinStepWidth(), 0.025));
      maxStepWidthSpinner.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getMaxStepWidth(), 0.05));
      turnStepWidth.setValueFactory(new DoubleSpinnerValueFactory(0.0, 1.0, initialParameters.getTurnStepWidth(), 0.05));
      turnMaxAngleInwardSpinner.setValueFactory(newAngleSpinnerValueFactory(0.0,
                                                                            Math.PI / 2.0,
                                                                            initialParameters.getTurnMaxAngleInward(),
                                                                            Math.toRadians(5.0)));
      turnMaxAngleOutwardSpinner.setValueFactory(newAngleSpinnerValueFactory(0.0,
                                                                             Math.PI / 2.0,
                                                                             initialParameters.getTurnMaxAngleOutward(),
                                                                             Math.toRadians(5.0)));

      stepParametersProperty.bindBidirectionalSwingHeight(swingHeightSlider.valueProperty());
      stepParametersProperty.bindBidirectionalSwingDuration(swingDurationSlider.valueProperty());
      stepParametersProperty.bindBidirectionalTransferDuration(transferDurationSlider.valueProperty());
      stepParametersProperty.bindBidirectionalMaxStepLength(maxStepLengthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalDefaultStepWidth(defaultStepWidthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalMinStepWidth(minStepWidthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalMaxStepWidth(maxStepWidthSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalTurnStepWidth(turnStepWidth.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalTurnMaxAngleInward(turnMaxAngleInwardSpinner.getValueFactory().valueProperty());
      stepParametersProperty.bindBidirectionalTurnMaxAngleOutward(turnMaxAngleOutwardSpinner.getValueFactory().valueProperty());

      messager.bindBidirectional(StepGeneratorJavaFXTopics.SteppingParameters, stepParametersProperty, true);
      messager.bindBidirectional(WalkingTrajectoryDuration, trajectoryDurationSlider.valueProperty(), createConverter(), true);
   }

   private PropertyToMessageTypeConverter<Double, Number> createConverter()
   {
      return new PropertyToMessageTypeConverter<Double, Number>()
      {
         @Override
         public Double convert(Number propertyValue)
         {
            return propertyValue.doubleValue();
         }

         @Override
         public Number interpret(Double messageContent)
         {
            return messageContent;
         }
      };
   }

   public static DoubleSpinnerValueFactory newAngleSpinnerValueFactory(double min, double max, double initialValue, double angleToStepBy)
   {
      DoubleSpinnerValueFactory doubleSpinnerValueFactory = new DoubleSpinnerValueFactory(min, max, initialValue, angleToStepBy);
      doubleSpinnerValueFactory.setConverter(StringConverterTools.radiansToRoundedDegrees());
      return doubleSpinnerValueFactory;
   }
}
