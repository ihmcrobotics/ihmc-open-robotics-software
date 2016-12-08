package us.ihmc.robotDataVisualizer.logger.lidar;

import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import us.ihmc.javaFXToolkit.TextFormatterTools;

public class LidarScanLogWriterControlPaneController
{
   @FXML
   private ToggleButton logDataToggleButton;
   @FXML
   private ToggleButton connectToggleButton;
   @FXML
   private TextField networkProcessorIPAdTextField;

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

      networkProcessorIPAdTextField.setTextFormatter(TextFormatterTools.ipAddressTextFormatter());
      networkProcessorIPAdTextField.setText(loggerController.networkProcessorAddressProperty().get());
   }

   @FXML
   private void clearScans()
   {
      lidarScanLogViewer.clearScans();
   }
}
