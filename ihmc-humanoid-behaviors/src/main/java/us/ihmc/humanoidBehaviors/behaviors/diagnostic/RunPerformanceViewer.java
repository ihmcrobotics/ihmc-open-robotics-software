package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import javafx.application.Application;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.fxml.Initializable;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.stage.Stage;

import java.net.URL;
import java.text.DecimalFormat;
import java.util.HashMap;
import java.util.ResourceBundle;

public class RunPerformanceViewer extends Application implements Initializable
{
   public SQLBehaviorDatabaseManager dataBase = new SQLBehaviorDatabaseManager();
   private ObservableList<Run> runsList = FXCollections.observableArrayList();
   private ObservableList<RunEvent> runEventsList = FXCollections.observableArrayList();
   private ObservableList<RunEvent> runEventsSummaryList = FXCollections.observableArrayList();
   private DecimalFormat format = new DecimalFormat("0.0");

   @FXML
   private TableView<Run> runsTableView;
   @FXML
   private TableColumn<Run, String> runTableColumn;
   @FXML
   private TableColumn<Run, String> operatorTableColumn;
   @FXML
   private TableColumn<Run, String> taskTableColumn;
   @FXML
   private TableColumn<Run, String> successTableColumn;
   @FXML
   private TableColumn<Run, String> notesTableColumn;
   @FXML
   private TextField numberOfRunsTextField;
   @FXML
   private DatePicker runsDate;
   @FXML
   private Label selectedRunIDLabel;
   @FXML
   private Label taskNameLabel;
   @FXML
   private Label totalTimeLabel;
   @FXML
   private Label operatorNameLabel;
   @FXML
   private Label successLabel;
   @FXML
   private TableView<RunEvent> eventsListTableView;
   @FXML
   private TableColumn<RunEvent, String> eventsListColumn;
   @FXML
   private TableColumn<RunEvent, String> eventsListTimeColumn;
   @FXML
   private TableView<RunEvent> summaryEventsListTableView;
   @FXML
   private TableColumn<RunEvent, String> summaryEventsListColumn;
   @FXML
   private TableColumn<RunEvent, String> summaryEventsListTimeColumn;


   public RunPerformanceViewer()
   {
   }

   @FXML
   void getAllRuns(ActionEvent event)
   {
      runsList.clear();
      runsList.addAll(dataBase.getAllRuns());
   }

   @FXML
   void getLastRuns(ActionEvent event)
   {

   }

   @FXML
   void getRunsByDate(ActionEvent event)
   {

   }

   @Override
   public void start(Stage stage) throws Exception
   {
      Parent root = FXMLLoader.load(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      Scene scene = new Scene(root, 800, 600);
      stage.setTitle(getClass().getSimpleName());
      stage.setScene(scene);
      stage.show();
   }

   @Override
   public void initialize(URL location, ResourceBundle resources)
   {
      runTableColumn.setCellValueFactory(new PropertyValueFactory<>("runID"));
      operatorTableColumn.setCellValueFactory(new PropertyValueFactory<>("operatorName"));
      taskTableColumn.setCellValueFactory(new PropertyValueFactory<>("taskName"));
      successTableColumn.setCellValueFactory(new PropertyValueFactory<>("successful"));
      notesTableColumn.setCellValueFactory(new PropertyValueFactory<>("notes"));

      runsTableView.setItems(runsList);

      runsTableView.setRowFactory(tv -> {
         TableRow<Run> row = new TableRow<>();
         row.setOnMouseClicked(event -> {
            if (event.getClickCount() == 1 && (!row.isEmpty()))
            {
               runEventsList.clear();
               runEventsSummaryList.clear();

               Run selectedItem = runsTableView.getSelectionModel().getSelectedItem();
               selectedRunIDLabel.setText("" + selectedItem.runID);
               operatorNameLabel.setText(selectedItem.getOperatorName());
               taskNameLabel.setText(selectedItem.getTaskName());
               successLabel.setText("" + selectedItem.isSuccessful());

               runEventsList.addAll(dataBase.getEventsForRun(selectedItem.getRunID()));

               HashMap<String, Float> eventSet = new HashMap<>();
               Float totalTime = 0.0f;
               for (RunEvent runEvent : runEventsList)
               {
                  totalTime += runEvent.getEventTimeInSeconds();

                  Float time = eventSet.get(runEvent.getEventName());
                  if (time == null)
                     time = new Float(0.0);

                  time = time + runEvent.getEventTimeInSeconds();
                  eventSet.put(runEvent.getEventName(), time);
               }

               for (String eventName : eventSet.keySet())
               {
                  RunEvent runEvent = new RunEvent(selectedItem.runID, eventName, eventSet.get(eventName), true);
                  runEventsSummaryList.add(runEvent);
               }

               totalTimeLabel.setText(format.format(totalTime));
            }
         });
         return row;
      });

      eventsListColumn.setCellValueFactory(new PropertyValueFactory<>("eventName"));
      eventsListTimeColumn.setCellValueFactory(new PropertyValueFactory<>("eventTimeInSeconds"));
      eventsListTableView.setItems(runEventsList);

      summaryEventsListColumn.setCellValueFactory(new PropertyValueFactory<>("eventName"));
      summaryEventsListTimeColumn.setCellValueFactory(new PropertyValueFactory<>("eventTimeInSeconds"));
      summaryEventsListTableView.setItems(runEventsSummaryList);
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
