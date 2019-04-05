package us.ihmc.avatar.joystickBasedJavaFXController;

import static us.ihmc.avatar.joystickBasedJavaFXController.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class StepGeneratorParametersPaneController
{
   @FXML
   private Slider swingHeightSlider;

   @FXML
   private Slider swingDurationSlider;

   @FXML
   private Slider transferDurationSlider;
   
   @FXML
   private Slider trajectoryDurationSlider;

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

      trajectoryDurationSlider.setValue(1.0);
      swingHeightSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      

      stepParametersProperty.bindBidirectionalSwingHeightThreshold(swingHeightSlider.valueProperty());
      stepParametersProperty.bindBidirectionalSwingDurationThreshold(swingDurationSlider.valueProperty());
      stepParametersProperty.bindBidirectionalTransferDurationThreshold(transferDurationSlider.valueProperty());
      
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
}
