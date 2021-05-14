package us.ihmc.footstepPlanning.ui.controllers;

import javafx.collections.FXCollections;
import javafx.fxml.FXML;
import javafx.scene.control.ListView;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.DataSet;

public class FootstepPlannerTestDashboardController
{
   private JavaFXMessager messager;

   @FXML
   private ListView<DataSet> testDashboardDataSets;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.TestDataSets,
                                     dataSets -> testDashboardDataSets.setItems(FXCollections.observableArrayList(dataSets)));

      testDashboardDataSets.getSelectionModel()
                           .selectedItemProperty()
                           .addListener((observable, oldValue, newValue) -> broadcastDataSetSelected(newValue));
   }

   private void broadcastDataSetSelected(DataSet dataSet)
   {
      new Thread(() -> messager.submitMessage(FootstepPlannerMessagerAPI.TestDataSetSelected, dataSet)).start();
   }
}
