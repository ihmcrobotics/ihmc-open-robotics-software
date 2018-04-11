package us.ihmc.atlas.joystickBasedStepping;

import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingSwingDuration;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingSwingHeight;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingTrajectoryDuration;
import static us.ihmc.atlas.joystickBasedStepping.StepGeneratorJavaFXTopics.WalkingTransferDuration;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
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

   public StepGeneratorParametersPaneController()
   {
   }

   public void initialize(JavaFXMessager messager, WalkingControllerParameters walkingControllerParameters)
   {
      swingHeightSlider.setLabelFormatter(StringConverterTools.metersToRoundedCentimeters());
      messager.bindBidirectional(WalkingSwingHeight, swingHeightSlider.valueProperty(), createConverter(), true);
      swingDurationSlider.setValue(walkingControllerParameters.getDefaultSwingTime());
      transferDurationSlider.setValue(walkingControllerParameters.getDefaultTransferTime());
      messager.bindBidirectional(WalkingSwingDuration, swingDurationSlider.valueProperty(), createConverter(), true);
      messager.bindBidirectional(WalkingTransferDuration, transferDurationSlider.valueProperty(), createConverter(), true);
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
