package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;

public class ImageProcessingAnchorPaneController
{
   private JavaFXMessager messager;
   @FXML private ToggleButton btnEnableStreaming;
   @FXML private Button btnTaskingSnapshot;
   @FXML private Button btnClearImageView;

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;
      btnTaskingSnapshot.setOnAction(new EventHandler<ActionEvent>()
      {
         @Override
         public void handle(ActionEvent event)
         {
            System.out.println("ImageProcessingAnchorPaneController true");
            messager.submitMessage(LidarImageFusionAPI.TakeSnapShot, true);
         }
      });

      messager.bindBidirectional(LidarImageFusionAPI.EnableStreaming, btnEnableStreaming.selectedProperty(), false);
   }

   public void clearImageView()
   {
      messager.submitMessage(LidarImageFusionAPI.ClearSnapShot, true);
   }
}