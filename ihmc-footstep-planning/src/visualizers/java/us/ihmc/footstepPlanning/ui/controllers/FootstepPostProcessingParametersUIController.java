package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.TableView;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.postProcessing.parameters.FootstepPostProcessingParametersBasics;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper.ParametersTableRow;

public class FootstepPostProcessingParametersUIController
{
   private JavaFXMessager messager;
   private FootstepPostProcessingParametersBasics postProcessingParameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;
   private StoredPropertyTableViewWrapper tableViewWrapper;

   @FXML
   private TableView<ParametersTableRow> parameterTable;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setPostProcessingParameters(FootstepPostProcessingParametersBasics parameters)
   {
      this.postProcessingParameters = parameters;
      this.javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(postProcessingParameters);
   }

   public void bindControls()
   {
      tableViewWrapper = new StoredPropertyTableViewWrapper(380.0, 260.0, 4, parameterTable, javaFXStoredPropertyMap);
      tableViewWrapper.setTableUpdatedCallback(() -> messager.submitMessage(FootstepPlannerMessagerAPI.PostProcessingParametersTopic, postProcessingParameters));
   }

   public void onPrimaryStageLoaded()
   {
      tableViewWrapper.removeHeader();
   }

   @FXML
   public void saveParameters()
   {
      postProcessingParameters.save();
   }
}
