package us.ihmc.avatar.heightMap;

import javafx.fxml.FXML;
import javafx.scene.control.TableView;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class HeightMapParametersUIController
{
   private JavaFXMessager messager;
   private HeightMapParameters parameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;
   private StoredPropertyTableViewWrapper tableViewWrapper;

   @FXML
   private TableView<StoredPropertyTableViewWrapper.ParametersTableRow> parameterTable;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setParameters(HeightMapParameters parameters)
   {
      this.parameters = parameters;
      this.javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(parameters);
   }

   public void bindControls()
   {
      tableViewWrapper = new StoredPropertyTableViewWrapper(360.0, 180.0, 4, parameterTable, javaFXStoredPropertyMap);
      tableViewWrapper.setTableUpdatedCallback(() -> messager.submitMessage(HeightMapMessagerAPI.parameters, parameters));
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
      messager.submitMessage(HeightMapMessagerAPI.parameters, parameters);
   }

}
