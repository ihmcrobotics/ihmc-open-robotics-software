package us.ihmc.avatar.heightMap;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.TableView;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyMap;
import us.ihmc.javafx.parameter.StoredPropertyTableViewWrapper;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.sensorProcessing.heightMap.HeightMapFilterParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class HeightMapParametersUIController
{
   private JavaFXMessager messager;
   private HeightMapParameters parameters;
   private HeightMapFilterParameters filterParameters;
   private JavaFXStoredPropertyMap javaFXStoredPropertyParameterMap;
   private JavaFXStoredPropertyMap javaFXStoredPropertyFilterParameterMap;
   private StoredPropertyTableViewWrapper parameterTableViewWrapper;
   private StoredPropertyTableViewWrapper filterParameterTableViewWrapper;

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
   @FXML
   private TableView<StoredPropertyTableViewWrapper.ParametersTableRow> filterParameterTable;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void setParameters(HeightMapParameters parameters)
   {
      this.parameters = parameters;
      this.javaFXStoredPropertyParameterMap = new JavaFXStoredPropertyMap(parameters);
   }

   public void setFilterParameters(HeightMapFilterParameters parameters)
   {
      this.filterParameters = parameters;
      this.javaFXStoredPropertyFilterParameterMap = new JavaFXStoredPropertyMap(parameters);
   }

   public void bindControls()
   {
      parameterTableViewWrapper = new StoredPropertyTableViewWrapper(340.0, 180.0, 2, parameterTable, javaFXStoredPropertyParameterMap, 3);
      parameterTableViewWrapper.setTableUpdatedCallback(() -> messager.submitMessage(HeightMapMessagerAPI.parameters, parameters));

      filterParameterTableViewWrapper = new StoredPropertyTableViewWrapper(340.0, 180.0, 2, filterParameterTable, javaFXStoredPropertyFilterParameterMap, 3);
      filterParameterTableViewWrapper.setTableUpdatedCallback(() -> messager.submitMessage(HeightMapMessagerAPI.filterParameters, filterParameters));

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
      parameterTableViewWrapper.removeHeader();
      filterParameterTableViewWrapper.removeHeader();
   }

   @FXML
   public void saveParameters()
   {
      parameters.save();
      filterParameters.save();
   }

   @FXML
   public void loadFile()
   {
      parameterTableViewWrapper.loadNewFile();
      filterParameterTableViewWrapper.loadNewFile();
      messager.submitMessage(HeightMapMessagerAPI.parameters, parameters);
      messager.submitMessage(HeightMapMessagerAPI.filterParameters, filterParameters);
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
