package us.ihmc.robotDataVisualizer.logger.lidar;

import java.io.IOException;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;

public class LidarScanLoggerUI extends Application
{
   @FXML
   private BorderPane mainPane;
   @FXML
   private Pane centerPane;
   @FXML
   private LidarScanLogWriterControlPaneController lidarScanLogWriterControlPaneController;
   @FXML
   private LidarScanLogReaderControlPaneController lidarScanLogReaderControlPaneController;

   private final LidarScanLoggerController controller = new LidarScanLoggerController();
   private final LidarScanLogViewer lidarScanLogViewer = new LidarScanLogViewer();

   public LidarScanLoggerUI() throws IOException
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader loader = new FXMLLoader();
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      loader.setController(this);
      loader.load();

      controller.setMainWindow(primaryStage);
      controller.enableNetworkProcessorClientProperty().set(true);
      controller.setLidarScanMessageConsumer(lidarScanLogViewer::renderLidarScanMessage);
      lidarScanLogViewer.start();
      lidarScanLogWriterControlPaneController.initialize(controller, lidarScanLogViewer);
      lidarScanLogReaderControlPaneController.initialize(controller);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.attachSubSceneTo(centerPane);
      view3dFactory.addNodeToView(lidarScanLogViewer.getRoot());
      
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setScene(new Scene(mainPane, 600, 400));
      primaryStage.show();
      primaryStage.setOnCloseRequest(event -> stop());
   }

   @Override
   public void stop()
   {
      try
      {
         controller.stop();
         lidarScanLogViewer.stop();
         Platform.exit();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      Application.launch(args);
   }
}
