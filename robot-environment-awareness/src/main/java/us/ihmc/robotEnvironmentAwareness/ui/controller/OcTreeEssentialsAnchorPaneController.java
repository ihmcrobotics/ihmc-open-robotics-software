package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.messager.javafx.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceNormalFilterParametersProperty;

public class OcTreeEssentialsAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableButton;
   @FXML
   private Button clearButton;
   // Main OcTree options
   @FXML
   private Slider depthSlider;
   @FXML
   private ComboBox<DisplayType> displayTypeComboBox;
   @FXML
   private ComboBox<ColoringType> coloringTypeComboBox;

   @FXML
   private Slider surfaceNormalLowerBoundSlider;
   @FXML
   private Slider surfaceNormalUpperBoundSlider;
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

   public OcTreeEssentialsAnchorPaneController()
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

      surfaceNormalUpperBoundSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
      surfaceNormalLowerBoundSlider.setLabelFormatter(StringConverterTools.radiansToRoundedDegrees());
   }

   @Override
   public void bindControls()
   {
      setupControls();

      uiMessager.bindBidirectionalGlobal(SegmentationModuleAPI.OcTreeEnable, enableButton.selectedProperty());

      surfaceNormalFilterParametersProperty.bindBidirectionalBounds(surfaceNormalUpperBoundSlider.valueProperty(),
                                                                    surfaceNormalLowerBoundSlider.valueProperty());
      surfaceNormalFilterParametersProperty.bindBidirectionalUseFilter(enableSurfaceNormalButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(SegmentationModuleAPI.SurfaceNormalFilterParameters, surfaceNormalFilterParametersProperty);

      load();
      uiMessager.bindBidirectionalInternal(SegmentationModuleAPI.UIOcTreeDepth, depthSlider.valueProperty(), numberToIntegerConverter, true);
      uiMessager.bindBidirectionalInternal(SegmentationModuleAPI.UIOcTreeDisplayType, displayTypeComboBox.valueProperty(), true);
      uiMessager.bindBidirectionalInternal(SegmentationModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox.valueProperty(), true);
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(SegmentationModuleAPI.OcTreeClear, true);
   }

   @FXML
   public void save()
   {
      uiMessager.submitStateRequestToModule(SegmentationModuleAPI.SaveUpdaterConfiguration);

      saveUIControlProperty(SegmentationModuleAPI.UIOcTreeDepth, depthSlider);
      saveUIControlProperty(SegmentationModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      saveUIControlProperty(SegmentationModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
   }

   public void load()
   {
      loadUIControlProperty(SegmentationModuleAPI.UIOcTreeDepth, depthSlider);
      loadUIControlProperty(SegmentationModuleAPI.UIOcTreeDisplayType, displayTypeComboBox);
      loadUIControlProperty(SegmentationModuleAPI.UIOcTreeColoringMode, coloringTypeComboBox);
   }
}
