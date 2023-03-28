package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.Tooltip;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.REAParametersMessageHelper;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.SurfaceNormalFilterParameters;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;

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
   private Spinner<Double> nodeLifetimeSpinner;
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
   private ToggleButton showStereoBufferButton;
   @FXML
   private Slider stereoBufferMessageSizeSlider;
   @FXML
   private Slider stereoBufferSizeSlider;
   @FXML
   private ToggleButton enableStereoBufferButton;
   @FXML
   private ToggleButton preserveStereoOcTreeHistoryButton;
   @FXML
   private ToggleButton showInputStereoFrameButton;

   @FXML
   private ToggleButton showDepthBufferButton;
   @FXML
   private ToggleButton enableDepthBufferButton;
   @FXML
   private Slider depthBufferMessageSizeSlider;
   @FXML
   private Slider depthBufferSizeSlider;
   @FXML
   private ToggleButton preserveDepthOcTreeHistoryButton;
   @FXML
   private ToggleButton showInputDepthCloudButton;

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
      // JFX doesn't like Double.POSITIVE_INFINITY for the upper bound, so just setting it to 1 day.
      nodeLifetimeSpinner.setValueFactory(new DoubleSpinnerValueFactory(0, 86400.0, 60.0, 5.0));
      ObservableList<DisplayType> displayTypeOptions = FXCollections.observableArrayList(DisplayType.values());
      displayTypeComboBox.setItems(displayTypeOptions);
      displayTypeComboBox.setValue(DisplayType.PLANE);
      ObservableList<ColoringType> coloringTypeOptions = FXCollections.observableArrayList(ColoringType.values());
      coloringTypeComboBox.setItems(coloringTypeOptions);
      coloringTypeComboBox.setValue(ColoringType.REGION);
      lidarBufferSizeSlider.setLabelFormatter(StringConverterTools.thousandRounding(true));

      Tooltip tooltip = new Tooltip();
      tooltip.setText("Press Clear Btn to apply the change of Octree resolution");
      resolutionSlider.setTooltip(tooltip);
   }

   private Topic<Boolean> ocTreeEnableTopic = REAModuleAPI.OcTreeEnable;

   public void setOcTreeEnableTopic(Topic<Boolean> ocTreeEnableTopic)
   {
      this.ocTreeEnableTopic = ocTreeEnableTopic;
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(ocTreeEnableTopic, enableButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeResolution, resolutionSlider.valueProperty(), numberToDoubleConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.OcTreeNodeLifetimeMillis,
                                         nodeLifetimeSpinner.getValueFactory().valueProperty(),
                                         millisecondToSecondTimeConverter());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarBufferEnable, enableLidarBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.LidarBufferOcTreeCapacity, lidarBufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton.selectedProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UILidarScanShow, showInputLidarScanButton.selectedProperty(), true);

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferEnable, enableStereoBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferMessageCapacity,
                                         stereoBufferMessageSizeSlider.valueProperty(),
                                         numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferSize, stereoBufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.StereoVisionBufferPreservingEnable, preserveStereoOcTreeHistoryButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIOcTreeShowStereoVisionBuffer, showStereoBufferButton.selectedProperty());
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIStereoVisionShow, showInputStereoFrameButton.selectedProperty(), true);

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.DepthCloudBufferEnable, enableDepthBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.DepthCloudBufferMessageCapacity, depthBufferMessageSizeSlider.valueProperty(),
                                         numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.DepthCloudBufferSize, depthBufferSizeSlider.valueProperty(), numberToIntegerConverter);
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.DepthCloudBufferPreservingEnable, preserveDepthOcTreeHistoryButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIOcTreeShowDepthCloudBuffer, showDepthBufferButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIDepthCloudShow, showInputDepthCloudButton.selectedProperty());

      load();
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDepth, depthSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox.valueProperty(), true);

      showInputLidarScanButton.selectedProperty().addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) ->
      {
         if (oldValue != newValue && !newValue)
         {
            uiMessager.submitMessageInternal(REAModuleAPI.UILidarScanClear, true);
            uiMessager.submitMessageInternal(REAModuleAPI.UIDepthCloudClear, true);
            uiMessager.submitMessageInternal(REAModuleAPI.UIStereoVisionClear, true);
         }
         // TODO add clearing the other scans
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
      saveUIControlProperty(REAModuleAPI.UIOcTreeShowStereoVisionBuffer, showStereoBufferButton);
      saveUIControlProperty(REAModuleAPI.UIOcTreeShowDepthCloudBuffer, showDepthBufferButton);
      saveUIControlProperty(REAModuleAPI.UIStereoVisionShow, showInputStereoFrameButton);
      saveUIControlProperty(REAModuleAPI.UIDepthCloudShow, showInputDepthCloudButton);
   }

   public void load()
   {
      loadUIControlProperty(REAModuleAPI.UIOcTreeDepth, depthSlider);
      loadUIControlProperty(REAModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowLidarBuffer, showLidarBufferButton);
      loadUIControlProperty(REAModuleAPI.UILidarScanShow, showInputLidarScanButton);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowStereoVisionBuffer, showStereoBufferButton);
      loadUIControlProperty(REAModuleAPI.UIOcTreeShowDepthCloudBuffer, showDepthBufferButton);
      loadUIControlProperty(REAModuleAPI.UIStereoVisionShow, showInputStereoFrameButton);
      loadUIControlProperty(REAModuleAPI.UIDepthCloudShow, showInputDepthCloudButton);
   }

   public void setParametersForStereo()
   {
      BoundingBoxParametersMessage boundingBoxMessage = new BoundingBoxParametersMessage();
      boundingBoxMessage.setMaxX(1.0f);
      boundingBoxMessage.setMinX(0.0f);
      boundingBoxMessage.setMaxY(1.0f);
      boundingBoxMessage.setMinY(-1.0f);
      boundingBoxMessage.setMaxZ(1.0f);
      boundingBoxMessage.setMinZ(-2.0f);

      uiMessager.broadcastMessage(REAModuleAPI.LidarBufferEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.StereoVisionBufferEnable, true);
      uiMessager.broadcastMessage(REAModuleAPI.DepthCloudBufferEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundingBoxMessage);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(7);
      uiMessager.broadcastMessage(REAModuleAPI.NormalEstimationParameters, normalEstimationParameters);

      PlanarRegionSegmentationParameters planarRegionSegmentationParameters = new PlanarRegionSegmentationParameters();
      planarRegionSegmentationParameters.setMaxDistanceFromPlane(0.03);
      planarRegionSegmentationParameters.setMaxAngleFromPlane(Math.toRadians(10.0));
      planarRegionSegmentationParameters.setMinRegionSize(150);
      uiMessager.broadcastMessage(REAModuleAPI.PlanarRegionsSegmentationParameters, planarRegionSegmentationParameters);

      SurfaceNormalFilterParameters surfaceNormalFilterParameters = new SurfaceNormalFilterParameters();
      surfaceNormalFilterParameters.setUseSurfaceNormalFilter(true);
      surfaceNormalFilterParameters.setSurfaceNormalLowerBound(Math.toRadians(-40.0));
      surfaceNormalFilterParameters.setSurfaceNormalUpperBound(Math.toRadians(40.0));
      uiMessager.broadcastMessage(REAModuleAPI.SurfaceNormalFilterParameters, surfaceNormalFilterParameters);

      PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
      polygonizerParameters.setConcaveHullThreshold(0.15);
      uiMessager.broadcastMessage(REAModuleAPI.PlanarRegionsPolygonizerParameters, REAParametersMessageHelper.convertToMessage(polygonizerParameters));
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
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxEnable, false);
      uiMessager.broadcastMessage(REAModuleAPI.OcTreeBoundingBoxParameters, boundindBoxMessage);
      uiMessager.broadcastMessage(REAModuleAPI.UIOcTreeDisplayType, DisplayType.HIDE);
   }

   private PropertyToMessageTypeConverter<Long, Double> millisecondToSecondTimeConverter()
   {
      return new PropertyToMessageTypeConverter<Long, Double>()
      {

         @Override
         public Long convert(Double lifetimeSeconds)
         {
            return (long) (1.0e3 * lifetimeSeconds);
         }

         @Override
         public Double interpret(Long lifetimeMillis)
         {
            return 1.0e-3 * lifetimeMillis;
         }
      };
   }
}
