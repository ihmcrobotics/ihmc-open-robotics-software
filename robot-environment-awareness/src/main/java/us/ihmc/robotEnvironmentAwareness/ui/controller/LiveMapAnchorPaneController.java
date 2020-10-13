package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.robotEnvironmentAwareness.communication.LiveMapModuleAPI;

public class LiveMapAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton enableMapFusion;
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
      uiMessager.bindBidirectionalGlobal(LiveMapModuleAPI.EnableMapFusion, enableMapFusion.selectedProperty());
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

   @FXML
   public void clearLocalizedMap()
   {
      uiMessager.broadcastMessage(LiveMapModuleAPI.ClearLocalizedMap, true);
   }
}
