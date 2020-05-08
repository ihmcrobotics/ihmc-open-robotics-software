package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.layout.Region;
import javafx.scene.text.Text;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.TopicListener;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerExceptionStackTrace;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTimings;

public class FootstepPlannerStatusBarController
{
   private JavaFXMessager messager;

   @FXML
   private Text sentRequestId;
   @FXML
   private Text receivedRequestId;
   @FXML
   private Button viewExceptionButton;

   @FXML
   private Text totalPlanTime;
   @FXML
   private Text bodyPathPlanTime;
   @FXML
   private Text bodyPathPlanResult;
   @FXML
   private Text stepPlanResult;
   @FXML
   private Text footstepPlanTime;
   @FXML
   private Text iterationsTaken;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.PlannerRequestId, new TextViewerListener<>(sentRequestId));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.ReceivedPlanId, new TextViewerListener<>(receivedRequestId));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.BodyPathPlanningResultTopic, new TextViewerListener<>(bodyPathPlanResult));
      messager.registerJavaFXSyncedTopicListener(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, new TextViewerListener<>(stepPlanResult));

      AtomicReference<String> stackTrace = messager.createInput(PlannerExceptionStackTrace, "No stack trace available");
      viewExceptionButton.setOnAction(e ->
                                      {
                                         TextArea textArea = new TextArea(stackTrace.toString());
                                         textArea.setEditable(false);

                                         Alert alert = new Alert(Alert.AlertType.NONE);
                                         alert.getButtonTypes().add(ButtonType.CLOSE);
                                         alert.setHeaderText("Planner Stack Trace");
                                         alert.setResizable(true);
                                         alert.getDialogPane().setMinSize(Region.USE_PREF_SIZE, Region.USE_PREF_SIZE);
                                         alert.getDialogPane().setContent(textArea);
                                         alert.show();
                                      });

      messager.registerTopicListener(PlannerTimings, timings ->
      {
         totalPlanTime.setText(String.format("%.2f", timings.getTotalElapsedSeconds()));
         bodyPathPlanTime.setText(String.format("%.2f", timings.getTimePlanningBodyPathSeconds()));
         footstepPlanTime.setText(String.format("%.2f", timings.getTimePlanningStepsSeconds()));
         iterationsTaken.setText(Long.toString(timings.getStepPlanningIterations()));
      });
   }

   private static class TextViewerListener<T> implements TopicListener<T>
   {
      private final Text textField;

      public TextViewerListener(Text textField)
      {
         this.textField = textField;
      }

      @Override
      public void receivedMessageForTopic(T messageContent)
      {
         if (messageContent != null)
            textField.setText(messageContent.toString());
      }
   }
}
