package us.ihmc.footstepPlanning.ui.controllers;

import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UIStepAdjustmentFrame;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.messager.TopicListener;
import us.ihmc.robotics.robotSide.RobotSide;

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

   @FXML
   private Label worldFrameLabel;
   @FXML
   private Label regionFrameLabel;

   @FXML
   private TableView<FootstepTableProperty> footstepPlanTable;

   private final ObservableList<FootstepTableProperty> footstepPlanTableItems = FXCollections.observableArrayList();

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

      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepPlanResponse ->
      {
         footstepPlanTableItems.clear();
         for (int i = 0; i < footstepPlanResponse.getFootstepDataList().size(); i++)
         {
            FootstepDataMessage footstepDataMessage = footstepPlanResponse.getFootstepDataList().get(i);
            footstepPlanTableItems.add(new FootstepTableProperty(i, footstepDataMessage));
         }
      });

      footstepPlanTable.setPrefWidth(240.0);
      footstepPlanTable.setPrefHeight(320.0);
      footstepPlanTable.getColumns().add(new TableColumn<>("Index"));
      footstepPlanTable.getColumns().add(new TableColumn<>("Side"));
      footstepPlanTable.getColumns().get(0).setCellValueFactory(new PropertyValueFactory<>("stepIndex"));
      footstepPlanTable.getColumns().get(1).setCellValueFactory(new PropertyValueFactory<>("side"));
      footstepPlanTable.getColumns().get(0).setPrefWidth(120.0);
      footstepPlanTable.getColumns().get(1).setPrefWidth(120.0);
      footstepPlanTable.setItems(footstepPlanTableItems);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ComputePath, c -> footstepPlanTableItems.clear());
      messager.registerTopicListener(FootstepPlannerMessagerAPI.GlobalReset, c -> footstepPlanTableItems.clear());

      footstepPlanTable.getSelectionModel().selectedItemProperty().addListener((observer, oldValue, newValue) ->
                                                                               {
                                                                                  if (newValue != null)
                                                                                  {
                                                                                     messager.submitMessage(FootstepPlannerMessagerAPI.SelectedFootstep,
                                                                                                            Pair.of(newValue.stepIndex,
                                                                                                                    newValue.footstepDataMessage));
                                                                                  }
                                                                               });

      messager.registerTopicListener(FootstepPlannerMessagerAPI.FootstepAdjustmentFrame, this::setFrame);
      setFrame(UIStepAdjustmentFrame.getDefault());
   }

   private final Border solidBorder = new Border(new BorderStroke(Color.BLACK.brighter(), BorderStrokeStyle.SOLID, null, new BorderWidths(1)));
   private final Border noBorder = new Border(new BorderStroke(Color.TRANSPARENT, BorderStrokeStyle.SOLID, null, new BorderWidths(1)));

   private void setFrame(UIStepAdjustmentFrame adjustmentFrame)
   {
      if (adjustmentFrame == UIStepAdjustmentFrame.WORLD)
      {
         worldFrameLabel.setBorder(solidBorder);
         regionFrameLabel.setBorder(noBorder);
      }
      else
      {
         regionFrameLabel.setBorder(solidBorder);
         worldFrameLabel.setBorder(noBorder);
      }
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

   public class FootstepTableProperty
   {
      private final int stepIndex;
      private final FootstepDataMessage footstepDataMessage;
      private final RobotSide side;

      public FootstepTableProperty(int stepIndex, FootstepDataMessage footstepDataMessage)
      {
         this.stepIndex = stepIndex;
         this.footstepDataMessage = footstepDataMessage;
         this.side = RobotSide.fromByte(footstepDataMessage.getRobotSide());
      }

      public int getStepIndex()
      {
         return stepIndex;
      }

      public RobotSide getSide()
      {
         return side;
      }
   }
}
