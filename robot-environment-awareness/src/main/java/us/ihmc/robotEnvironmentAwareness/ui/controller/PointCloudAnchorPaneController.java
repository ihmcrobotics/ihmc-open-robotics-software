package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
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
   @FXML
   private Spinner<Integer> sizeOfPointCloudSpinner;

   private static final int maximumSizeOfPointCloud = 200000;
   private static final int minimumSizeOfPointCloud = 1000;
   public static final int initialSizeOfPointCloud = 5000;

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
         return new Double(newValue.intValue());
      }
   };

   public PointCloudAnchorPaneController()
   {
   }

   public void bindControls()
   {
      load();
      sizeOfPointCloudSpinner.setValueFactory(createNumberOfPointsValueFactory(initialSizeOfPointCloud));
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, enableLidarButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIStereoVisionShow, enableStereoButton.selectedProperty(), true);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIStereoVisionSize, sizeOfPointCloudSpinner.getValueFactory().valueProperty());
   }

   @FXML
   public void clearLidar()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
   }

   @FXML
   public void clearStereo()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIStereoVisionClear, true);
   }

   @FXML
   public void save()
   {
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      saveUIControlProperty(REAModuleAPI.UIStereoVisionShow, enableStereoButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, enableLidarButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanSize, scanHistorySizeSlider);
      loadUIControlProperty(REAModuleAPI.UIStereoVisionShow, enableStereoButton);
   }

   private IntegerSpinnerValueFactory createNumberOfPointsValueFactory(int initialValue)
   {
      int min = minimumSizeOfPointCloud;
      int max = maximumSizeOfPointCloud;
      int amountToStepBy = minimumSizeOfPointCloud;
      return new IntegerSpinnerValueFactory(min, max, initialValue, amountToStepBy);
   }
}
