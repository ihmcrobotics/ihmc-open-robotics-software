package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.RandomICPSLAMParametersProperty;

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

   private final RandomICPSLAMParametersProperty ihmcSLAMParametersProperty = new RandomICPSLAMParametersProperty(this, "ihmcSLAMParameters");

   public SLAMAnchorPaneController()
   {

   }

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
      uiMessager.broadcastMessage(REAModuleAPI.SLAMEnable, true);
      uiMessager.broadcastMessage(REAModuleAPI.ShowLatestFrame, true);
      uiMessager.broadcastMessage(REAModuleAPI.ShowSLAMOctreeMap, true);
      uiMessager.broadcastMessage(REAModuleAPI.ShowSLAMSensorTrajectory, true);
      uiMessager.broadcastMessage(REAModuleAPI.ShowPlanarRegionsMap, true);
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(REAModuleAPI.SLAMClear, true);
      uiMessager.broadcastMessage(REAModuleAPI.SLAMVizClear, true);
   }
}
