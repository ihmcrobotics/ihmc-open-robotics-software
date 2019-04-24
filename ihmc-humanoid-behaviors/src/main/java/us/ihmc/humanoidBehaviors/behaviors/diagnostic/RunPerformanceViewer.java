package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import javafx.application.Application;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.DatePicker;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.stage.Stage;

import java.util.ArrayList;

public class RunPerformanceViewer extends Application
{
   public SQLBehaviorDatabaseManager dataBase = new SQLBehaviorDatabaseManager();
   private ObservableList<Run> runsList = FXCollections.observableArrayList();

   @FXML
   private TableView<Run> runsTable = new TableView<>();
   @FXML
   private TableColumn<Run, String> runTableColumn = new TableColumn<>();
   @FXML
   private TableColumn<Run, String> operatorTableColumn = new TableColumn<>();;
   @FXML
   private TableColumn<Run, String> taskTableColumn = new TableColumn<>();;
   @FXML
   private TableColumn<Run, String> successTableColumn = new TableColumn<>();;
   @FXML
   private TableColumn<Run, String> notesTableColumn = new TableColumn<>();;
   @FXML
   private TextField numberOfRunsTextField;
   @FXML
   private DatePicker runsDate;

   public RunPerformanceViewer()
   {
   }

   @FXML
   void getAllRuns(ActionEvent event)
   {
      runsList.clear();
      ArrayList<Run> allRuns = dataBase.getAllRuns();
      for (Run run : allRuns)
      {
         System.out.println(run);
      }
      runsList.addAll(allRuns);
      System.out.println("runsList = " + runsList.size());

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
      Scene scene = new Scene(root, 600, 476);
      stage.setTitle(getClass().getSimpleName());
      stage.setScene(scene);
      stage.show();

      runTableColumn.setCellValueFactory(new PropertyValueFactory<>("runID"));
      operatorTableColumn.setCellValueFactory(new PropertyValueFactory<>("operatorID"));
      taskTableColumn.setCellValueFactory(new PropertyValueFactory<>("taskID"));
      successTableColumn.setCellValueFactory(new PropertyValueFactory<>("sucess"));
      notesTableColumn.setCellValueFactory(new PropertyValueFactory<>("notes"));

      runsTable.setItems(runsList);
      Run temp = new Run(99, 89);
      runsList.add(temp);
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
