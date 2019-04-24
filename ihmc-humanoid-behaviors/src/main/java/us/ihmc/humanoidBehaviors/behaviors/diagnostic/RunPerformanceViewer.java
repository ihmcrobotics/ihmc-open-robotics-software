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
import javafx.scene.control.DatePicker;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.stage.Stage;

import java.net.URL;
import java.util.ArrayList;
import java.util.ResourceBundle;

public class RunPerformanceViewer extends Application implements Initializable
{
   public SQLBehaviorDatabaseManager dataBase = new SQLBehaviorDatabaseManager();
   private ObservableList<Run> runsList = FXCollections.observableArrayList();

   @FXML
   private TableView<Run> runsTable;
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
   }

   @Override
   public void initialize(URL location, ResourceBundle resources)
   {
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
