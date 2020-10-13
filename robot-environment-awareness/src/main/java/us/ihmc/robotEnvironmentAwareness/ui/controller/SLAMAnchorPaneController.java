package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.fxml.FXML;
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
   private TextField frameComputationTime;

   @FXML
   private TextField slamComputationTime;

   @FXML
   private TextField averageComputationTime;

   @FXML
   private TextField listenerComputationTime;

   @FXML
   private TextField totalComputationTime;

   @FXML
   private ToggleButton latestFrameEnable;

   @FXML
   private ToggleButton octreeMapEnable;

   @FXML
   private ToggleButton showNormal;

   @FXML
   private ToggleButton sensorFrameEnable;

   @FXML
   private Label stationaryFlag;
   @FXML
   private Label velocityLimitFlag;
   
   @FXML
   private TextField speed;

   private final BooleanProperty sensorMovingProperty = new SimpleBooleanProperty(this, "sensorMovingProperty", false);
   private final BooleanProperty velocityLimitProperty = new SimpleBooleanProperty(this, "velocityLimitProperty", false);


   private final SurfaceElementICPSLAMParametersProperty ihmcSLAMParametersProperty = new SurfaceElementICPSLAMParametersProperty(this, "ihmcSLAMParameters");

   public SLAMAnchorPaneController()
   {

   }

   private void updateSensorStatusViz(boolean notMoving)
   {
      Platform.runLater(() ->
      {
         if (!notMoving)
            stationaryFlag.setStyle("-fx-background-color: red;");
         else
            stationaryFlag.setStyle("-fx-background-color: green;");
      });
   }


   private void updateVelocityLimitStatus(boolean notMoving)
   {
      Platform.runLater(() ->
      {
         if (!notMoving)
            velocityLimitFlag.setStyle("-fx-background-color: red;");
         else
            velocityLimitFlag.setStyle("-fx-background-color: green;");
      });
   }


   @Override
   public void bindControls()
   {
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SLAMEnable, enableSLAMButton.selectedProperty());

      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.QueuedBuffers, queuedBufferSize.textProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.FrameComputationTime, frameComputationTime.textProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SLAMComputationTime, slamComputationTime.textProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.AverageComputationTime, averageComputationTime.textProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.ListenerComputationTime, listenerComputationTime.textProperty());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.TotalComputationTime, totalComputationTime.textProperty());

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
      
      uiMessager.bindPropertyToTopic(SLAMModuleAPI.VelocityLimitStatus, velocityLimitProperty);
      updateVelocityLimitStatus(false);
      velocityLimitProperty.addListener((o, oldValue, newValue) ->
      {
         if (newValue != oldValue)
            updateVelocityLimitStatus(newValue);
      });

      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.SensorSpeed, speed.textProperty());
   }

   private void initializeSetup()
   {
      uiMessager.broadcastMessage(SLAMModuleAPI.SLAMEnable, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowLatestFrame, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowSLAMOctreeMap, true);
      uiMessager.broadcastMessage(SLAMModuleAPI.ShowSLAMSensorTrajectory, true);
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
