package us.ihmc.quadrupedUI.uiControllers;

import java.util.concurrent.atomic.AtomicReference;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedUI.QuadrupedUIMessagerAPI;

public class RobotControlTabController
{
   @FXML
   private TextField currentStateViewer;

   @FXML
   private Button sitDownButton;

   @FXML
   private Button standUpButton;

   @FXML
   private Button stopWalkingButton;

   @FXML
   private CheckBox enablePoseTeleopControl;

   @FXML
   private CheckBox enableStepTeleopControl;

   @FXML
   private CheckBox enableHeightTeleopControl;

   @FXML
   private CheckBox enableJoystickControl;


   private AtomicReference<HighLevelControllerName> currentControllerState;

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentControllerState = messager.createInput(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, null);

      messager.addTopicListener(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, this::validateBodyTopic);
      messager.addTopicListener(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, this::validateWalkingTopic);
   }

   private void validateBodyTopic(boolean request)
   {
      if (request)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
         if (currentControllerState.get() != null && currentControllerState.get() != HighLevelControllerName.WALKING)
            messager.submitMessage(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, false);
      }
   }

   private void validateWalkingTopic(boolean request)
   {
      if (request)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, false);
         if (currentControllerState.get() != null && currentControllerState.get() != HighLevelControllerName.WALKING)
            messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
      }
      else
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.AbortWalkingTopic, true);
      }
   }

   public void requestStopWalking()
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
      messager.submitMessage(QuadrupedUIMessagerAPI.AbortWalkingTopic, true);
   }

   public void requestSitDown()
   {
      if (currentControllerState.get() == HighLevelControllerName.WALKING)
         messager.submitMessage(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, HighLevelControllerName.EXIT_WALKING);
      else
         messager.submitMessage(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, QuadrupedControllerManager.sitDownStateName);
   }

   public void requestStandUp()
   {
      messager.submitMessage(QuadrupedUIMessagerAPI.DesiredControllerNameTopic, HighLevelControllerName.STAND_PREP_STATE);
   }

   public void bindControls()
   {
      messager.addFXTopicListener(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, new TextViewerListener<>(currentStateViewer));
      messager.bindBidirectional(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, enablePoseTeleopControl.selectedProperty(), true);
      messager.bindBidirectional(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, enableStepTeleopControl.selectedProperty(), true);
      messager.bindBidirectional(QuadrupedUIMessagerAPI.EnableHeightTeleopTopic, enableHeightTeleopControl.selectedProperty(), true);
      messager.bindBidirectional(QuadrupedUIMessagerAPI.EnableJoystickTopic, enableJoystickControl.selectedProperty(), true);
   }

   private class TextViewerListener<T> implements TopicListener<T>
   {
      private final TextField textField;
      public TextViewerListener(TextField textField)
      {
         this.textField = textField;
      }

      public void receivedMessageForTopic(T messageContent)
      {
         if (messageContent != null)
            textField.promptTextProperty().setValue(messageContent.toString());
      }
   }
}
