package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.viewer.REAMeshViewer.SourceType;

public class PointCloudAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private Slider scanHistorySizeSlider;
   @FXML
   private ComboBox<SourceType> sourceTypeComboBox;

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

   public PointCloudAnchorPaneController()
   {
   }

   public void setupControls()
   {
      ObservableList<SourceType> sourceTypeOptions = FXCollections.observableArrayList(SourceType.values());
      sourceTypeComboBox.setItems(sourceTypeOptions);
      sourceTypeComboBox.setValue(SourceType.LidarScan);
   }

   public void bindControls()
   {
      setupControls();
      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, enableButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanSourceType, sourceTypeComboBox.valueProperty(), true);

   }

   @FXML
   public void clear()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
   }

   @FXML
   public void save()
   {
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, enableButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      saveUIControlProperty(REAModuleAPI.UILidarScanSourceType, sourceTypeComboBox);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, enableButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      loadUIControlProperty(REAModuleAPI.UILidarScanSourceType, sourceTypeComboBox);
   }
}
