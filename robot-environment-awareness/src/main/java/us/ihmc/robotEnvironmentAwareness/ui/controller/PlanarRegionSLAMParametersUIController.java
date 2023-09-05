package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.TableView;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper.ParametersTableRow;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapModuleAPI;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;

public class PlanarRegionSLAMParametersUIController extends REABasicUIController
{
   private StoredPropertyTableViewWrapper tableViewWrapper;
   private PlanarRegionSLAMParameters slamParameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;

   @FXML
   private TableView<ParametersTableRow> parameterTable;

   public void setupParameters()
   {
      slamParameters = new PlanarRegionSLAMParameters("ForLiveMap");
      javaFXStoredPropertyMap = new JavaFXStoredPropertyMap(slamParameters);
   }

   public void bindControls()
   {
      tableViewWrapper = new StoredPropertyTableViewWrapper(380.0, 260.0, 4, parameterTable, javaFXStoredPropertyMap, 5);
      tableViewWrapper.setTableUpdatedCallback(() -> uiMessager.submitMessageToModule(LiveMapModuleAPI.PlanarRegionsSLAMParameters, slamParameters.getAllAsStrings()));

      // set messager updates to update all stored properties and select JavaFX properties
      uiMessager.registerTopicListener(LiveMapModuleAPI.PlanarRegionsSLAMParameters, parameters ->
      {
         slamParameters.setAllFromStrings(parameters);
         javaFXStoredPropertyMap.copyStoredToJavaFX();
      });
   }

   public void onPrimaryStageLoaded()
   {
      tableViewWrapper.removeHeader();
   }

   @FXML
   public void load()
   {
      slamParameters.load();
   }

   @FXML
   public void save()
   {
      slamParameters.save();
   }
}
