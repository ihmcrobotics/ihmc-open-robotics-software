package us.ihmc.avatar.heightMap;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TableView;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class HeightMapParametersUIController
{
   private JavaFXMessager messager;
   private HeightMapParameters parameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyMap;
   private StoredPropertyTableViewWrapper tableViewWrapper;

   @FXML
   private Spinner<Integer> publishFreq;
   @FXML
   private Spinner<Double> gridCenterX;
   @FXML
   private Spinner<Double> gridCenterY;
   @FXML
   private CheckBox enableUpdates;
   @FXML
   private Spinner<Integer> snapTestId;

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
      tableViewWrapper = new StoredPropertyTableViewWrapper(340.0, 180.0, 2, parameterTable, javaFXStoredPropertyMap, 3);
      tableViewWrapper.setTableUpdatedCallback(() -> messager.submitMessage(HeightMapMessagerAPI.parameters, parameters));

      publishFreq.setValueFactory(new SpinnerValueFactory.IntegerSpinnerValueFactory(1, Integer.MAX_VALUE));
      gridCenterX.setValueFactory(createGridCenterFactory());
      gridCenterY.setValueFactory(createGridCenterFactory());
      snapTestId.setValueFactory(new SpinnerValueFactory.IntegerSpinnerValueFactory(-1, Integer.MAX_VALUE));
      snapTestId.getValueFactory().setValue(-1);
      snapTestId.valueProperty().addListener((obs, vOld, vNew) -> messager.submitMessage(HeightMapMessagerAPI.SnapTestId, snapTestId.getValue()));

      messager.bindBidirectional(HeightMapMessagerAPI.PublishFrequency, publishFreq.getValueFactory().valueProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.GridCenterX, gridCenterX.getValueFactory().valueProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.GridCenterY, gridCenterY.getValueFactory().valueProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.EnableUpdates, enableUpdates.selectedProperty(), false);
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

   @FXML
   public void export()
   {
      messager.submitMessage(HeightMapMessagerAPI.Export, true);
   }

   @FXML
   public void importHeightMap()
   {
      messager.submitMessage(HeightMapMessagerAPI.Import, true);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createGridCenterFactory()
   {
      double min = -Double.MAX_VALUE;
      double max = Double.MAX_VALUE;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

}
