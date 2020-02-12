package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.IhmcSLAMParametersProperty;

public class SLAMAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableSLAMButton;

   @FXML
   private TextField queuedBufferSize;

   @FXML
   private TextField slamStatus;

   @FXML
   private ToggleButton latestFrameEnable;

   @FXML
   private ToggleButton octreeMapEnable;

   @FXML
   private ToggleButton sensorFrameEnable;

   @FXML
   private ToggleButton planarRegionsEnable;

   @FXML
   private Slider sourcePointsSlider;

   @FXML
   private Slider searchingSizeSlider;

   @FXML
   private Slider minimumOverlappedRatioSlider;

   @FXML
   private Slider windowMarginSlider;

   @FXML
   private Slider minimumInliersRatioSlider;

   private final IhmcSLAMParametersProperty ihmcSLAMParametersProperty = new IhmcSLAMParametersProperty(this, "ihmcSLAMParameters");

   public SLAMAnchorPaneController()
   {

   }

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

   @Override
   public void bindControls()
   {
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMEnable, enableSLAMButton.selectedProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.QueuedBuffers, queuedBufferSize.textProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMStatus, slamStatus.textProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowLatestFrame, latestFrameEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowSLAMOctreeMap, octreeMapEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowSLAMSensorTrajectory, sensorFrameEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.ShowPlanarRegionsMap, planarRegionsEnable.selectedProperty());

      ihmcSLAMParametersProperty.bindBidirectionalNumberOfSourcePoints(sourcePointsSlider.valueProperty());
      ihmcSLAMParametersProperty.bindBidirectionalMaximumICPSearchingSize(searchingSizeSlider.valueProperty());
      ihmcSLAMParametersProperty.bindBidirectionalMinimumOverlappedRatio(minimumOverlappedRatioSlider.valueProperty());
      ihmcSLAMParametersProperty.bindBidirectionalWindowSize(windowMarginSlider.valueProperty());
      ihmcSLAMParametersProperty.bindBidirectionalMinimumInliersRatioOfKeyFrame(minimumInliersRatioSlider.valueProperty());

      uiMessager.bindBidirectionalGlobal(REAModuleAPI.SLAMParameters, ihmcSLAMParametersProperty);

      initializeSetup();
   }

   private void initializeSetup()
   {
      uiMessager.submitMessageToModule(REAModuleAPI.SLAMEnable, true);
      uiMessager.submitMessageToModule(REAModuleAPI.ShowLatestFrame, true);
      uiMessager.submitMessageToModule(REAModuleAPI.ShowSLAMOctreeMap, true);
      uiMessager.submitMessageToModule(REAModuleAPI.ShowSLAMSensorTrajectory, true);
      uiMessager.submitMessageToModule(REAModuleAPI.ShowPlanarRegionsMap, true);

      uiMessager.submitMessageInternal(REAModuleAPI.SLAMEnable, true);
      uiMessager.submitMessageInternal(REAModuleAPI.ShowLatestFrame, true);
      uiMessager.submitMessageInternal(REAModuleAPI.ShowSLAMOctreeMap, true);
      uiMessager.submitMessageInternal(REAModuleAPI.ShowSLAMSensorTrajectory, true);
      uiMessager.submitMessageInternal(REAModuleAPI.ShowPlanarRegionsMap, true);
   }

   @FXML
   public void clear()
   {
      uiMessager.submitMessageToModule(REAModuleAPI.SLAMClear, true);
      uiMessager.submitMessageInternal(REAModuleAPI.SLAMClear, true);
      
      uiMessager.submitMessageToModule(REAModuleAPI.SLAMVizClear, true);
      uiMessager.submitMessageInternal(REAModuleAPI.SLAMVizClear, true);
   }

   @FXML
   public void initialize()
   {
      attachREAMessager(PointCloudAnchorPaneController.uiStaticMessager);
      bindControls();
   }
}
