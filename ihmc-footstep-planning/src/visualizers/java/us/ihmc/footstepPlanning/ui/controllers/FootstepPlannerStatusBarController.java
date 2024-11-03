package us.ihmc.footstepPlanning.ui.controllers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerExceptionStackTrace;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTimings;

import java.text.NumberFormat;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.FootstepDataMessage;
import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Alert;
import javafx.scene.control.Button;
import javafx.scene.control.ButtonType;
import javafx.scene.control.Label;
import javafx.scene.control.Spinner;
import javafx.scene.control.TableCell;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.layout.Border;
import javafx.scene.layout.BorderStroke;
import javafx.scene.layout.BorderStrokeStyle;
import javafx.scene.layout.BorderWidths;
import javafx.scene.layout.Region;
import javafx.scene.paint.Color;
import javafx.scene.text.Text;
import javafx.util.StringConverter;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.communication.UIStepAdjustmentFrame;
import us.ihmc.footstepPlanning.ui.controllers.FootstepPlannerStatusBarController.FootstepResponseTableRow;
import us.ihmc.messager.TopicListener;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.trajectories.TrajectoryType;

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
   private Text pathIterationsTaken;
   @FXML
   private Text bodyPathPlanResult;
   @FXML
   private Text stepPlanResult;
   @FXML
   private Text footstepPlanTime;
   @FXML
   private Text stepIterationsTaken;

   @FXML
   private Label worldFrameLabel;
   @FXML
   private Label regionFrameLabel;

   @FXML
   private TableView<FootstepResponseTableRow> footstepPlanTable;

   private final ObservableList<FootstepResponseTableRow> footstepPlanTableItems = FXCollections.observableArrayList();

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.addFXTopicListener(FootstepPlannerMessagerAPI.PlannerRequestId, new TextViewerListener<>(sentRequestId));
      messager.addFXTopicListener(FootstepPlannerMessagerAPI.ReceivedPlanId, new TextViewerListener<>(receivedRequestId));
      messager.addFXTopicListener(FootstepPlannerMessagerAPI.BodyPathPlanningResultTopic, new TextViewerListener<>(bodyPathPlanResult));
      messager.addFXTopicListener(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, new TextViewerListener<>(stepPlanResult));

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

      messager.addTopicListener(PlannerTimings, timings ->
      {
         totalPlanTime.setText(String.format("%.2f", timings.getTotalElapsedSeconds()));
         bodyPathPlanTime.setText(String.format("%.2f", timings.getTimePlanningBodyPathSeconds()));
         footstepPlanTime.setText(String.format("%.2f", timings.getTimePlanningStepsSeconds()));
         pathIterationsTaken.setText(Long.toString(timings.getPathPlanningIterations()));
         stepIterationsTaken.setText(Long.toString(timings.getStepPlanningIterations()));
      });

      messager.addTopicListener(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepPlanResponse ->
      {
         footstepPlanTableItems.clear();
         for (int i = 0; i < footstepPlanResponse.getFootstepDataList().size(); i++)
         {
            FootstepDataMessage footstepDataMessage = footstepPlanResponse.getFootstepDataList().get(i);
            footstepPlanTableItems.add(new FootstepResponseTableRow(i, footstepDataMessage));
         }
      });

      TableColumn<FootstepResponseTableRow, SwingTimeCellValue> swingTimeColumn = new TableColumn<>("Swing Time");
      swingTimeColumn.setPrefWidth(100.0);
      swingTimeColumn.setCellValueFactory(tableRow -> new ReadOnlyObjectWrapper<>(new SwingTimeCellValue(tableRow.getValue())));
      swingTimeColumn.setCellFactory(param -> new SwingTimeCell());

      footstepPlanTable.setPrefWidth(320.0);
      footstepPlanTable.setPrefHeight(360.0);
      footstepPlanTable.getColumns().add(new TableColumn<>("Index"));
      footstepPlanTable.getColumns().add(new TableColumn<>("Side"));
      footstepPlanTable.getColumns().add(swingTimeColumn);
      footstepPlanTable.getColumns().add(new TableColumn<>("Traj Type"));
      footstepPlanTable.getColumns().get(0).setCellValueFactory(new PropertyValueFactory<>("stepIndex"));
      footstepPlanTable.getColumns().get(1).setCellValueFactory(new PropertyValueFactory<>("side"));
      footstepPlanTable.getColumns().get(3).setCellValueFactory(new PropertyValueFactory<>("trajectoryType"));
      footstepPlanTable.getColumns().get(0).setPrefWidth(60.0);
      footstepPlanTable.getColumns().get(1).setPrefWidth(60.0);
      footstepPlanTable.getColumns().get(3).setPrefWidth(100.0);

      footstepPlanTable.setItems(footstepPlanTableItems);

      messager.addTopicListener(FootstepPlannerMessagerAPI.ComputePath, c -> footstepPlanTableItems.clear());
      messager.addTopicListener(FootstepPlannerMessagerAPI.GlobalReset, c -> footstepPlanTableItems.clear());

      footstepPlanTable.getSelectionModel().selectedItemProperty().addListener((observer, oldValue, newValue) ->
                                                                               {
                                                                                  if (newValue != null)
                                                                                  {
                                                                                     messager.submitMessage(FootstepPlannerMessagerAPI.SelectedFootstep,
                                                                                                            Pair.of(newValue.stepIndex,
                                                                                                                    newValue.footstepDataMessage));
                                                                                  }
                                                                               });

      messager.addTopicListener(FootstepPlannerMessagerAPI.FootstepAdjustmentFrame, this::setFrame);
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

   private class SwingTimeCellValue
   {
      private final int stepIndex;
      private final double initialSwingTime;

      public SwingTimeCellValue(FootstepResponseTableRow tableRow)
      {
         this.stepIndex = tableRow.getStepIndex();
         this.initialSwingTime = tableRow.getSwingTime();
      }
   }

   private static final NumberFormat swingTimeNumberFormat = NumberFormat.getInstance();

   static
   {
      swingTimeNumberFormat.setMinimumIntegerDigits(1);
      swingTimeNumberFormat.setMinimumFractionDigits(3);
      swingTimeNumberFormat.setMaximumFractionDigits(3);
   }

   public static class DoubleStringConverter extends StringConverter<Double>
   {
      @Override
      public String toString(Double object)
      {
         if (object == null)
            return swingTimeNumberFormat.format(0.0);
         else if (!Double.isFinite(object))
            return Double.toString(object);
         else
            return swingTimeNumberFormat.format(object);
      }

      @Override
      public Double fromString(String string)
      {
         return Double.parseDouble(string);
      }
   }

   private class SwingTimeCell extends TableCell<FootstepResponseTableRow, SwingTimeCellValue>
   {
      private final Spinner<Double> spinner = new Spinner<>(-Double.MAX_VALUE, Double.MAX_VALUE, 0.0, 0.1);

      public SwingTimeCell()
      {
         spinner.getEditor().setTextFormatter(new TextFormatter<>(new DoubleStringConverter()));
         spinner.getValueFactory().setConverter(new DoubleStringConverter());
      }

      @Override
      protected void updateItem(SwingTimeCellValue tableCell, boolean empty)
      {
         super.updateItem(tableCell, empty);

         if (tableCell != null)
         {
            spinner.setEditable(true);
            spinner.getValueFactory()
                   .valueProperty()
                   .addListener(((observable, oldValue, newValue) -> messager.submitMessage(FootstepPlannerMessagerAPI.OverrideSpecificSwingTime,
                                                                                            Pair.of(tableCell.stepIndex, newValue))));
            spinner.getValueFactory().valueProperty().setValue(tableCell.initialSwingTime);
            setGraphic(spinner);
         }
         else
         {
            setGraphic(null);
         }
      }
   }

   public class FootstepResponseTableRow
   {
      private final int stepIndex;
      private final FootstepDataMessage footstepDataMessage;
      private final RobotSide side;
      private final TrajectoryType trajectoryType;

      public FootstepResponseTableRow(int stepIndex, FootstepDataMessage footstepDataMessage)
      {
         this.stepIndex = stepIndex;
         this.footstepDataMessage = footstepDataMessage;
         this.side = RobotSide.fromByte(footstepDataMessage.getRobotSide());
         this.trajectoryType = TrajectoryType.fromByte(footstepDataMessage.getTrajectoryType());
      }

      public int getStepIndex()
      {
         return stepIndex;
      }

      public RobotSide getSide()
      {
         return side;
      }

      public double getSwingTime()
      {
         return footstepDataMessage.getSwingDuration();
      }

      public TrajectoryType getTrajectoryType()
      {
         return trajectoryType;
      }
   }
}
