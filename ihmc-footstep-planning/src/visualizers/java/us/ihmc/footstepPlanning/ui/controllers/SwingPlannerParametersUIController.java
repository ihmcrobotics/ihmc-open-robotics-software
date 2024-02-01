package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.TableView;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper.ParametersTableRow;
import us.ihmc.messager.javafx.JavaFXMessager;

public class SwingPlannerParametersUIController
{
   private JavaFXMessager messager;
   private SwingPlannerParametersBasics parameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;
   private StoredPropertyTableViewWrapper tableViewWrapper;

   @FXML
   private TableView<ParametersTableRow> parameterTable;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setParameters(SwingPlannerParametersBasics parameters)
   {
      this.parameters = parameters;
      this.javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(parameters);
   }

   public void bindControls()
   {
      tableViewWrapper = new StoredPropertyTableViewWrapper(380.0, 260.0, 4, parameterTable, javaFXStoredPropertyMap, 5);
      tableViewWrapper.setTableUpdatedCallback(() -> messager.submitMessage(FootstepPlannerMessagerAPI.SwingPlannerParameters, parameters));
   }

   public void onPrimaryStageLoaded()
   {
      tableViewWrapper.removeHeader();
   }

   @FXML
   public void saveParameters()
   {
      parameters.save();
   }

   @FXML
   public void loadFile()
   {
      tableViewWrapper.loadNewFile();
      messager.submitMessage(FootstepPlannerMessagerAPI.SwingPlannerParameters, parameters);
   }
}
