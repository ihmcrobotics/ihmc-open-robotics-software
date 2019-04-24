package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import javafx.application.Application;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.DatePicker;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;

import javax.swing.*;

public class RunPerformanceViewer extends Application
{
   //   public SQLBehaviorDatabaseManager dataBase = new SQLBehaviorDatabaseManager();

   public RunPerformanceViewer()
   {
   }

   @FXML
   private TableView<?> runsTable;

   @FXML
   private TextField numberOfRunsTextField;

   @FXML
   private DatePicker runsDate;

   @FXML
   void getAllRuns(ActionEvent event) {

   }

   @FXML
   void getLastRuns(ActionEvent event) {

   }

   @FXML
   void getRunsByDate(ActionEvent event) {

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

   public static void main(String[] args)
   {
      launch(args);
   }
}
