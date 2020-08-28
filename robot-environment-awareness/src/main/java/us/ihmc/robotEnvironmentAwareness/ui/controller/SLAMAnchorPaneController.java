package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceElementICPSLAMParametersProperty;

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
   private CheckBox showNormal;

   @FXML
   private ToggleButton sensorFrameEnable;

   @FXML
   private Label stationaryFlag;

   private final BooleanProperty sensorMovingProperty = new SimpleBooleanProperty(this, "sensorMovingProperty", false);
   private final SurfaceElementICPSLAMParametersProperty ihmcSLAMParametersProperty = new SurfaceElementICPSLAMParametersProperty(this, "ihmcSLAMParameters");

   public SLAMAnchorPaneController()
   {

   }

   private void updateSensorStatusViz(boolean moving)
   {
      Platform.runLater(() ->
      {
         if (moving)
            stationaryFlag.setStyle("-fx-background-color: red;");
         else
            stationaryFlag.setStyle("-fx-background-color: green;");
      });
   }

   @Override
   public void bindControls()
   {
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SLAMEnable, enableSLAMButton.selectedProperty());

      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.QueuedBuffers, queuedBufferSize.textProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SLAMStatus, slamStatus.textProperty());

      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.ShowLatestFrame, latestFrameEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.ShowSLAMOctreeMap, octreeMapEnable.selectedProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.ShowSLAMOctreeNormalMap, showNormal.selectedProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.ShowSLAMSensorTrajectory, sensorFrameEnable.selectedProperty());

      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SLAMParameters, ihmcSLAMParametersProperty);

      initializeSetup();

      uiMessager.bindPropertyToTopic(SLAMModuleAPI.SensorStatus, sensorMovingProperty);
      updateSensorStatusViz(false);
      sensorMovingProperty.addListener((o, oldValue, newValue) ->
      {
         if (newValue != oldValue)
            updateSensorStatusViz(newValue);
      });
   }

   private void initializeSetup()
   {
      uiMessager.broadcastMessage(SLAMModuleAPI.SLAMEnable, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowLatestFrame, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowSLAMOctreeMap, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowSLAMSensorTrajectory, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowPlanarRegionsMap, true);
   }

   @FXML
   public void clear()
   {
      uiMessager.broadcastMessage(SLAMModuleAPI.SLAMClear, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.SLAMVizClear, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.SensorPoseHistoryClear, true);
   }

   @FXML
   public void clearFootsteps()
   {
      uiMessager.broadcastMessage(SLAMModuleAPI.ClearFootstepDataViz, true);
   }
}
