package us.ihmc.avatar.heightMap;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TableView;
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
   private Spinner<Double> maxHeight;
   @FXML
   private Spinner<Double> xPosition;
   @FXML
   private Spinner<Double> yPosition;
   @FXML
   private Spinner<Double> zPosition;

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

      double initialValue = 0.4;
      maxHeight.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, initialValue, 0.1));

      xPosition.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, -2.0, 0.1));
      yPosition.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, -1.0, 0.1));
      zPosition.setValueFactory(new SpinnerValueFactory.DoubleSpinnerValueFactory(-Double.MAX_VALUE, Double.MAX_VALUE, 0.6, 0.1));

      messager.bindBidirectional(HeightMapMessagerAPI.PublishFrequency, publishFreq.getValueFactory().valueProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.GridCenterX, gridCenterX.getValueFactory().valueProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.GridCenterY, gridCenterY.getValueFactory().valueProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.EnableUpdates, enableUpdates.selectedProperty(), false);
      messager.bindBidirectional(HeightMapMessagerAPI.MaxHeight, maxHeight.getValueFactory().valueProperty(), true);
      messager.bindBidirectional(HeightMapMessagerAPI.xPosition, xPosition.getValueFactory().valueProperty(), true);
      messager.bindBidirectional(HeightMapMessagerAPI.yPosition, yPosition.getValueFactory().valueProperty(), true);
      messager.bindBidirectional(HeightMapMessagerAPI.zPosition, zPosition.getValueFactory().valueProperty(), true);
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

   @FXML
   public void clear()
   {
      messager.submitMessage(HeightMapMessagerAPI.Clear, true);
   }

   private SpinnerValueFactory.DoubleSpinnerValueFactory createGridCenterFactory()
   {
      double min = -Double.MAX_VALUE;
      double max = Double.MAX_VALUE;
      double amountToStepBy = 0.1;
      return new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, 0.0, amountToStepBy);
   }

}
