package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.BoundingBoxParametersProperty;

public class LIDARFilterAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableBoundingBoxButton;
   @FXML
   private ToggleButton showBoundingBoxButton;
   @FXML
   private Spinner<Double> boundingBoxMinXSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxXSpinner;
   @FXML
   private Spinner<Double> boundingBoxMinYSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxYSpinner;
   @FXML
   private Spinner<Double> boundingBoxMinZSpinner;
   @FXML
   private Spinner<Double> boundingBoxMaxZSpinner;

   private final BoundingBoxParametersProperty boundingBoxParametersProperty = new BoundingBoxParametersProperty(this, "boundingBoxParameters");

   @FXML
   private Slider lidarMinRange;
   @FXML
   private Slider lidarMaxRange;

   private final PropertyToMessageTypeConverter<Double, Number> converter = new PropertyToMessageTypeConverter<Double, Number>()
   {

      @Override
      public Double convert(Number propertyValue)
      {
         return propertyValue.doubleValue();
      }

      @Override
      public Number interpret(Double messageContent)
      {
         return messageContent.doubleValue();
      }
   };

   public LIDARFilterAnchorPaneController()
   {
   }

   private void setupControls()
   {
      boundingBoxMinXSpinner.setValueFactory(createBoundingBoxValueFactory(0.0));
      boundingBoxMaxXSpinner.setValueFactory(createBoundingBoxValueFactory(5.0));
      boundingBoxMinYSpinner.setValueFactory(createBoundingBoxValueFactory(-2.0));
      boundingBoxMaxYSpinner.setValueFactory(createBoundingBoxValueFactory(2.0));
      boundingBoxMinZSpinner.setValueFactory(createBoundingBoxValueFactory(-1.0));
      boundingBoxMaxZSpinner.setValueFactory(createBoundingBoxValueFactory(1.0));
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBoundingBoxEnable, enableBoundingBoxButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarMinRange, lidarMinRange.valueProperty(), converter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarMaxRange, lidarMaxRange.valueProperty(), converter);

      boundingBoxParametersProperty.binBidirectionalMinX(boundingBoxMinXSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMinY(boundingBoxMinYSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMinZ(boundingBoxMinZSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxX(boundingBoxMaxXSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxY(boundingBoxMaxYSpinner.getValueFactory().valueProperty());
      boundingBoxParametersProperty.binBidirectionalMaxZ(boundingBoxMaxZSpinner.getValueFactory().valueProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBoxParametersProperty);

      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeBoundingBoxShow, showBoundingBoxButton.selectedProperty(), true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveMainUpdaterConfiguration);
      saveUIControlProperty(REAModuleAPI.UIOcTreeBoundingBoxShow, showBoundingBoxButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UIOcTreeBoundingBoxShow, showBoundingBoxButton);
   }

   private DoubleSpinnerValueFactory createBoundingBoxValueFactory(double initialValue)
   {
      double min = -100.0;
      double max = 100.0;
      double amountToStepBy = 0.1;
      return new DoubleSpinnerValueFactory(min, max, initialValue, amountToStepBy);
   }
}
