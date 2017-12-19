package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.robotEnvironmentAwareness.communication.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;

public class OcTreeBasicsAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private Button clearButton;
   @FXML
   private Slider depthSlider;
   @FXML
   private ComboBox<DisplayType> displayTypeComboBox;
   @FXML
   private ComboBox<ColoringType> coloringTypeComboBox;
   @FXML
   private ToggleButton showBufferButton;
   @FXML
   private Slider bufferSizeSlider;
   @FXML
   private ToggleButton showInputScanButton;

   private final PropertyToMessageTypeConverter<Integer, Number> numberToIntegerConverter = new PropertyToMessageTypeConverter<Integer, Number>()
   {
      @Override
      public Integer convert(Number propertyValue)
      {
         return propertyValue.intValue();
      }

      @Override
      public Number interpret(Integer newValue)
      {
         return new Double(newValue.doubleValue());
      }
   };

   public OcTreeBasicsAnchorPaneController()
   {
   }

   public void setupControls()
   {
      ObservableList<DisplayType> displayTypeOptions = FXCollections.observableArrayList(DisplayType.values());
      displayTypeComboBox.setItems(displayTypeOptions);
      displayTypeComboBox.setValue(DisplayType.PLANE);
      ObservableList<ColoringType> coloringTypeOptions = FXCollections.observableArrayList(ColoringType.values());
      coloringTypeComboBox.setItems(coloringTypeOptions);
      coloringTypeComboBox.setValue(ColoringType.REGION);
      bufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeEnable, enableButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeBufferSize, bufferSizeSlider.valueProperty(), numberToIntegerConverter);

      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDepth, depthSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeShowBuffer, showBufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, showInputScanButton.selectedProperty(), true);
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeClear, true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveMainUpdaterConfiguration);
      uiMessager.submitStateRequestToModule(REAModuleAPI.SaveBufferConfiguration);

      saveUIControlProperty(REAModuleAPI.UIOcTreeDepth, depthSlider);
      saveUIControlProperty(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      saveUIControlProperty(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
      saveUIControlProperty(REAModuleAPI.UIOcTreeShowBuffer, showBufferButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, showInputScanButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UIOcTreeDepth, depthSlider);
      loadUIControlProperty(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowBuffer, showBufferButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, showInputScanButton);
   }
}
