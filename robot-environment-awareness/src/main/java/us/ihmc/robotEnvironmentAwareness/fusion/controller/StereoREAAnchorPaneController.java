package us.ihmc.robotEnvironmentAwareness.fusion.controller;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;

public class StereoREAAnchorPaneController
{
   private JavaFXMessager messager;

   @FXML
   private TextField tfSeedLabel;

   @FXML
   private Button btnLoadData;

   @FXML
   private Button btnClearViz;

   @FXML
   private Button btnAll;

   @FXML
   private Button btnPropagate;

   @FXML
   private Button btnPlanarRegion;

   @FXML
   private Button btnEndToEnd;

   public void initialize(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void load()
   {
      messager.submitMessage(LidarImageFusionAPI.LoadData, true);
   }

   public void clear()
   {
      messager.submitMessage(LidarImageFusionAPI.ClearViz, true);
   }

   public void all()
   {
      messager.submitMessage(LidarImageFusionAPI.VisualizeAll, true);
   }

   public void propagate()
   {
      messager.submitMessage(LidarImageFusionAPI.Seed, Integer.parseInt(tfSeedLabel.getText()));
      messager.submitMessage(LidarImageFusionAPI.Propagate, true);
   }

   public void planar()
   {
      messager.submitMessage(LidarImageFusionAPI.Seed, Integer.parseInt(tfSeedLabel.getText()));
      messager.submitMessage(LidarImageFusionAPI.PlanarRegion, true);
   }

   public void endToEnd()
   {
      messager.submitMessage(LidarImageFusionAPI.EndToEnd, true);
   }
}
