package us.ihmc.quadrupedUI.uiControllers;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TextField;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.TopicListener;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedUI.QuadrupedUIMessagerAPI;

import java.util.concurrent.atomic.AtomicReference;

public class MainTabController
{
   @FXML
   private TextField currentStateViewer;

   @FXML
   private Button sitDownButton;

   @FXML
   private Button standUpButton;

   @FXML
   private CheckBox enableBodyPoseControl;

   @FXML
   private CheckBox enableFootstepControl;

   private AtomicReference<HighLevelControllerName> currentControllerState;

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentControllerState = messager.createInput(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, HighLevelControllerName.WALKING);

      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, this::validateBodyTopic);
      messager.registerTopicListener(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, this::validateWalkingTopic);
   }

   private void validateBodyTopic(boolean request)
   {
      if (request)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
         if (currentControllerState.get() != HighLevelControllerName.WALKING)
            messager.submitMessage(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, false);
      }
   }

   private void validateWalkingTopic(boolean request)
   {
      if (request)
      {
         messager.submitMessage(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, false);
         if (currentControllerState.get() != HighLevelControllerName.WALKING)
            messager.submitMessage(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, false);
      }
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
      messager.registerJavaFXSyncedTopicListener(QuadrupedUIMessagerAPI.CurrentControllerNameTopic, new TextViewerListener<>(currentStateViewer));
      messager.bindBidirectional(QuadrupedUIMessagerAPI.EnableBodyTeleopTopic, enableBodyPoseControl.selectedProperty(), true);
      messager.bindBidirectional(QuadrupedUIMessagerAPI.EnableStepTeleopTopic, enableFootstepControl.selectedProperty(), true);
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
