package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceNormalFilterParametersProperty;

public class OcTreeBasicsAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private Button clearButton;
   // Main OcTree options
   @FXML
   private Slider depthSlider;
   @FXML
   private Slider resolutionSlider;
   @FXML
   private ComboBox<DisplayType> displayTypeComboBox;
   @FXML
   private ComboBox<ColoringType> coloringTypeComboBox;
   // Lidar buffer options
   @FXML
   private Slider lidarBufferSizeSlider;
   @FXML
   private ToggleButton enableLidarBufferButton;
   @FXML
   private ToggleButton showLidarBufferButton;
   @FXML
   private ToggleButton showInputLidarScanButton;
   // Stereo vision buffer options
   @FXML
   private Slider stereoBufferMessageSizeSlider;
   @FXML
   private Slider stereoBufferSizeSlider;
   @FXML
   private ToggleButton enableStereoBufferButton;
   @FXML
   private ToggleButton enableDepthCloudBufferButton;
   @FXML
   private Slider surfaceNormalLowerBoundSlider;
   @FXML
   private Slider surfaceNormalUpperBoundSlider;
   @FXML
   private ToggleButton preserveOcTreeHistoryButton;
   @FXML
   private ToggleButton enableSurfaceNormalButton;

   private final SurfaceNormalFilterParametersProperty surfaceNormalFilterParametersProperty = new SurfaceNormalFilterParametersProperty(this,
                                                                                                                                         "surfaceNormalFilterParametersProperty");

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
   
   private final PropertyToMessageTypeConverter<Double, Number> numberToDoubleConverter = new PropertyToMessageTypeConverter<Double, Number>()
   {
      @Override
      public Double convert(Number propertyValue)
      {
         return propertyValue.doubleValue();
      }

      @Override
      public Number interpret(Double newValue)
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
      lidarBufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));
      
      surfaceNormalUpperBoundSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
      surfaceNormalLowerBoundSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
      
      Tooltip tooltip = new Tooltip();
      tooltip.setText("Press Clear Btn to apply the change of Octree resolution");
      resolutionSlider.setTooltip(tooltip);
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeEnable, enableButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeResolution, resolutionSlider.valueProperty(), numberToDoubleConverter);

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarBufferEnable, enableLidarBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarBufferOcTreeCapacity, lidarBufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferEnable, enableStereoBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.DepthCloudBufferEnable, enableDepthCloudBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferMessageCapacity, stereoBufferMessageSizeSlider.valueProperty(),
                                         numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferSize, stereoBufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferPreservingEnable, preserveOcTreeHistoryButton.selectedProperty());

      surfaceNormalFilterParametersProperty.bindBidirectionalBounds(surfaceNormalUpperBoundSlider.valueProperty(),
                                                                    surfaceNormalLowerBoundSlider.valueProperty());
      surfaceNormalFilterParametersProperty.bindBidirectionalUseFilter(enableSurfaceNormalButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SurfaceNormalFilterParameters, surfaceNormalFilterParametersProperty);

      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDepth, depthSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, showInputLidarScanButton.selectedProperty(), true);

      showInputLidarScanButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> {
         if (oldValue != newValue && !newValue)
            uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
      });

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
      saveUIControlProperty(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton);
      saveUIControlProperty(REAModuleAPI.UILidarScanShow, showInputLidarScanButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UIOcTreeDepth, depthSlider);
      loadUIControlProperty(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, showInputLidarScanButton);
   }
   
   public void setParametersForStereo()
   {
      BoundingBoxParametersMessage boundindBoxMessage = new BoundingBoxParametersMessage();
      boundindBoxMessage.setMaxX(1.0f);
      boundindBoxMessage.setMinX(0.0f);
      boundindBoxMessage.setMaxY(1.0f);
      boundindBoxMessage.setMinY(-1.0f);
      boundindBoxMessage.setMaxZ(1.0f);
      boundindBoxMessage.setMinZ(-2.0f);
      
      uiMessager.broadcastMessage(REAModuleAPI.LidarBufferEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      uiMessager.broadcastMessage(REAModuleAPI.DepthCloudBufferEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundindBoxMessage);
      uiMessager.broadcastMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
   }
   
   public void setParametersForDepth()
   {
      BoundingBoxParametersMessage boundindBoxMessage = new BoundingBoxParametersMessage();
      boundindBoxMessage.setMaxX(1.0f);
      boundindBoxMessage.setMinX(0.0f);
      boundindBoxMessage.setMaxY(1.0f);
      boundindBoxMessage.setMinY(-1.0f);
      boundindBoxMessage.setMaxZ(1.0f);
      boundindBoxMessage.setMinZ(-2.0f);
      
      uiMessager.broadcastMessage(REAModuleAPI.LidarBufferEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.StereoVisionBufferEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.DepthCloudBufferEnable, true);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundindBoxMessage);
      uiMessager.broadcastMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
   }
}
