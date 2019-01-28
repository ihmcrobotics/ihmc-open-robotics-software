package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class PointCloudAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableLidarButton;
   @FXML
   private Slider scanHistorySizeSlider;
   @FXML
   private ToggleButton enableStereoButton;

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

   public void bindControls()
   {
      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, enableLidarButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIStereoVisionPointCloudShow, enableStereoButton.selectedProperty(), true);

   }

   @FXML
   public void clearLidar()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
   }

   @FXML
   public void clearStereo()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIStereoVisionPointCloudClear, true);
   }

   @FXML
   public void save()
   {
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      saveUIControlProperty(REAModuleAPI.UIStereoVisionPointCloudShow, enableStereoButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      loadUIControlProperty(REAModuleAPI.UIStereoVisionPointCloudShow, enableStereoButton);
   }
}
