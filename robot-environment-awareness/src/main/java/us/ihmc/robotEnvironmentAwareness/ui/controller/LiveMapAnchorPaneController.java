package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Slider;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.StringConverterTools;
import us.ihmc.javaFXToolkit.messager.MessageBidirectionalBinding.PropertyToMessageTypeConverter;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.SegmentationModuleAPI;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.ColoringType;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;
import us.ihmc.robotEnvironmentAwareness.ui.properties.SurfaceNormalFilterParametersProperty;

public class LiveMapAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableLidarButton;
   @FXML
   private ToggleButton enableRealSenseButton;

   public LiveMapAnchorPaneController()
   {
   }

   @Override
   public void bindControls()
   {
      uiMessager.bindBidirectionalGlobal(LiveMapModuleAPI.EnableLidar, enableLidarButton.selectedProperty());
      uiMessager.bindBidirectionalGlobal(LiveMapModuleAPI.EnableRealSense, enableRealSenseButton.selectedProperty());
   }

   @FXML
   public void clearLidar()
   {
      uiMessager.broadcastMessage(LiveMapModuleAPI.ClearLidar, true);
   }

   @FXML
   public void clearRealSense()
   {
      uiMessager.broadcastMessage(LiveMapModuleAPI.ClearRealSense, true);
   }
}
