package us.ihmc.robotDataVisualizer.logger.lidar;

import javafx.beans.value.ChangeListener;
import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;

public class LidarScanLogReaderControlPaneController
{
   @FXML
   private ToggleButton startReadingToggleButton;
   @FXML
   private ToggleButton pauseToggleButton;
   @FXML
   private ToggleButton waitForListenerToggleButton;

   private LidarScanLoggerController loggerController;

   public LidarScanLogReaderControlPaneController()
   {
   }

   public void initialize(LidarScanLoggerController loggerController)
   {
      this.loggerController = loggerController;
      startReadingToggleButton.selectedProperty().bindBidirectional(loggerController.readingProperty());
      startReadingToggleButton.selectedProperty().addListener(
            (ChangeListener<Boolean>) (observable, oldValue, newValue) -> startReadingToggleButton.setText(newValue ? "Stop Reading" : "Start Reading"));
      pauseToggleButton.selectedProperty().bindBidirectional(loggerController.pauseReadingProperty());
      waitForListenerToggleButton.selectedProperty().bindBidirectional(loggerController.waitForListenerWhenReadingProperty());
   }

   @FXML
   private void reloadLog()
   {
      loggerController.reloadLogFile();
   }
}
