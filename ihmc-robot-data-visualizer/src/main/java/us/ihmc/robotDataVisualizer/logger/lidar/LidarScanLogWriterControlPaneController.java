package us.ihmc.robotDataVisualizer.logger.lidar;

import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;

public class LidarScanLogWriterControlPaneController
{
   @FXML
   private ToggleButton logDataToggleButton;
   @FXML
   private ToggleButton connectToggleButton;
   @FXML
   private TextField lidarScanTopicNameTextField;

   private LidarScanLogViewer lidarScanLogViewer;

   public LidarScanLogWriterControlPaneController()
   {
   }

   public void initialize(LidarScanLoggerController loggerController, LidarScanLogViewer lidarScanLogViewer)
   {
      this.lidarScanLogViewer = lidarScanLogViewer;
      logDataToggleButton.selectedProperty().bindBidirectional(loggerController.writingProperty());
      logDataToggleButton.selectedProperty().addListener(
            (ChangeListener<Boolean>) (observable, oldValue, newValue) -> logDataToggleButton.setText(newValue ? "Stop Writing" : "Start Writing"));

      connectToggleButton.selectedProperty().bindBidirectional(loggerController.enableNetworkProcessorClientProperty());
      connectToggleButton.selectedProperty()
            .addListener((ChangeListener<Boolean>) (observable, oldValue, newValue) -> connectToggleButton.setText(newValue ? "Disconnect" : "Connect"));

//      logDataToggleButton.selectedProperty().bindBidirectional(connectToggleButton.disableProperty());
//      connectToggleButton.selectedProperty().bindBidirectional(logDataToggleButton.disableProperty());

      lidarScanTopicNameTextField.setText(loggerController.lidarScanTopicNameProperty().get());
   }

   @FXML
   private void clearScans()
   {
      lidarScanLogViewer.clearScans();
   }
}
