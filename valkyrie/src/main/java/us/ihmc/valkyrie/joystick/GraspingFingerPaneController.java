package us.ihmc.valkyrie.joystick;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.Slider;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;

public class GraspingFingerPaneController
{
   @FXML
   private Slider sliderLeftThumbRoll;

   @FXML
   private Slider sliderRightThumbRoll;

   @FXML
   private Slider sliderLeftThumb;
   
   @FXML
   private Slider sliderLeftThumb2;

   @FXML
   private Slider sliderLeftIndex;

   @FXML
   private Slider sliderLeftMiddle;

   @FXML
   private Slider sliderLeftPinky;

   @FXML
   private Slider sliderRightThumb;
   
   @FXML
   private Slider sliderRightThumb2;

   @FXML
   private Slider sliderRightIndex;

   @FXML
   private Slider sliderRightMiddle;

   @FXML
   private Slider sliderRightPinky;

   @FXML
   private Button btnSendLeftFingerMessage;

   @FXML
   private Button btnSendRightFingerMessage;

   public GraspingFingerPaneController()
   {

   }

   public void initialize(JavaFXMessager messager)
   {
      messager.bindBidirectional(GraspingJavaFXTopics.LeftThumbRoll, sliderLeftThumbRoll.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.RightThumbRoll, sliderRightThumbRoll.valueProperty(), createConverter(), true);

      messager.bindBidirectional(GraspingJavaFXTopics.LeftThumb, sliderLeftThumb.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.LeftThumb2, sliderLeftThumb2.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.LeftIndex, sliderLeftIndex.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.LeftMiddle, sliderLeftMiddle.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.LeftPinky, sliderLeftPinky.valueProperty(), createConverter(), true);

      messager.bindBidirectional(GraspingJavaFXTopics.RightThumb, sliderRightThumb.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.RightThumb2, sliderRightThumb2.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.RightIndex, sliderRightIndex.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.RightMiddle, sliderRightMiddle.valueProperty(), createConverter(), true);
      messager.bindBidirectional(GraspingJavaFXTopics.RightPinky, sliderRightPinky.valueProperty(), createConverter(), true);

      btnSendLeftFingerMessage.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            messager.submitMessage(GraspingJavaFXTopics.LeftSendMessage, true);
         }
      });

      btnSendRightFingerMessage.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            messager.submitMessage(GraspingJavaFXTopics.RightSendMessage, true);
         }
      });
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