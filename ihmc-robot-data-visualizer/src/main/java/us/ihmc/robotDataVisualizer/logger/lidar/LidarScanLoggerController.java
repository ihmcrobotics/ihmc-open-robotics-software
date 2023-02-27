package us.ihmc.robotDataVisualizer.logger.lidar;

import java.io.File;
import java.io.IOException;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;

import perception_msgs.msg.dds.LidarScanMessage;
import javafx.beans.InvalidationListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ObservableValue;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Window;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;

public class LidarScanLoggerController
{
   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "lidar_scan_logger");

   private final LidarScanLogWriter logWriter = new LidarScanLogWriter(ros2Node);
   private final LidarScanLogReader logReader = new LidarScanLogReader();

   private Window mainWindow;
   // TODO make it part of the ui
   private NetworkPorts portToOpen = NetworkPorts.REA_MODULE_PORT;

   public LidarScanLoggerController()
   {
   }

   public void setLidarScanMessageConsumer(PacketConsumer<LidarScanMessage> consumer)
   {
      logWriter.setLidarScanMessageConsumer(consumer);
      logReader.setLidarScanMessageConsumer(consumer);
   }

   public void reloadLogFile()
   {
      logReader.reloadLogFile();
   }

   public void setMainWindow(Window mainWindow)
   {
      this.mainWindow = mainWindow;
   }

   public void stop()
   {
      writingProperty().set(false);
      enableNetworkProcessorClientProperty().set(false);

      readingProperty().set(false);
      enableLoggerServerProperty().set(false);

      logWriter.stopExecutor();
      logReader.stopExecutor();
   }

   private BooleanProperty writingProperty;

   public BooleanProperty writingProperty()
   {
      if (writingProperty == null)
      {
         writingProperty = new SimpleBooleanProperty(this, "writeLogProperty", false);
         writingProperty.addListener(this::toggleWriting);
      }

      return writingProperty;
   }

   private BooleanProperty readingProperty;

   public BooleanProperty readingProperty()
   {
      if (readingProperty == null)
      {
         readingProperty = new SimpleBooleanProperty(this, "readLogProperty", false);
         readingProperty.addListener(this::toggleReading);
      }

      return readingProperty;
   }

   private StringProperty lidarScanTopicNameProperty;

   public StringProperty lidarScanTopicNameProperty()
   {
      if (lidarScanTopicNameProperty == null)
      {
         lidarScanTopicNameProperty = new SimpleStringProperty(this, "networkProcessorAddressProperty", "/ihmc/lidar_scan");
      }
      return lidarScanTopicNameProperty;
   }

   private BooleanProperty enableNetworkProcessorClientProperty;

   public BooleanProperty enableNetworkProcessorClientProperty()
   {
      if (enableNetworkProcessorClientProperty == null)
      {
         enableNetworkProcessorClientProperty = new SimpleBooleanProperty(this, "enableNetworkProcessorClientProperty", false);
         enableNetworkProcessorClientProperty.addListener(this::toggleConnectionToNetworkProcessor);
      }

      return enableNetworkProcessorClientProperty;
   }

   private BooleanProperty enableLoggerServerProperty;

   public BooleanProperty enableLoggerServerProperty()
   {
      if (enableLoggerServerProperty == null)
      {
         enableLoggerServerProperty = new SimpleBooleanProperty(this, "enableLoggerServerProperty", false);
         enableLoggerServerProperty.addListener(this::toggleLoggerServer);
      }

      return enableLoggerServerProperty;
   }

   private BooleanProperty waitForListenerWhenReadingProperty;

   public BooleanProperty waitForListenerWhenReadingProperty()
   {
      if (waitForListenerWhenReadingProperty == null)
      {
         waitForListenerWhenReadingProperty = new SimpleBooleanProperty(this, "waitForListenerWhenReadingProperty", false);
         waitForListenerWhenReadingProperty
               .addListener((InvalidationListener) observable -> logReader.waitForListener(waitForListenerWhenReadingProperty.get()));
      }

      return waitForListenerWhenReadingProperty;
   }

   private BooleanProperty pauseReadingProperty;

   public BooleanProperty pauseReadingProperty()
   {
      if (pauseReadingProperty == null)
      {
         pauseReadingProperty = new SimpleBooleanProperty(this, "pauseReadingProperty", false);
         pauseReadingProperty.addListener(this::togglePause);
      }

      return pauseReadingProperty;
   }

   private void toggleWriting(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
   {
      if (oldValue == newValue)
         return;

      if (newValue)
         startWriting();
      else
         logWriter.stopWriting();
   }

   private void startWriting()
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setInitialFileName(getDate() + "LogName.ihmcLidarLog");
      File file = fileChooser.showSaveDialog(mainWindow);

      if (file != null)
      {
         readingProperty().set(false);
         enableLoggerServerProperty().set(false);
         logWriter.startWriting(file);
      }
      else
      {
         writingProperty().set(false);
      }
   }

   private String getDate()
   {
      DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_");
      LocalDateTime now = LocalDateTime.now();
      return now.format(dateTimeFormatter);
   }

   private void toggleReading(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
   {
      if (oldValue == newValue)
         return;

      if (newValue)
         startReading();
      else
         logReader.stopReading();
   }

   private void startReading()
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setSelectedExtensionFilter(new ExtensionFilter("IHMC lidar scan log file.", ".ihmcLidarLog"));
      File file = fileChooser.showOpenDialog(mainWindow);

      if (file != null)
      {
         writingProperty().set(false);
         enableNetworkProcessorClientProperty().set(false);
         enableLoggerServerProperty().set(true);
         logReader.startReading(file);
      }
      else
      {
         readingProperty().set(false);
      }
   }

   private void toggleConnectionToNetworkProcessor(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
   {
      if (oldValue == newValue)
         return;

      if (newValue)
      {
         try
         {
            logWriter.connectToNetworkProcessor(lidarScanTopicNameProperty().get());
         }
         catch (IOException e)
         {
            e.printStackTrace();
            enableNetworkProcessorClientProperty().set(false);
         }
      }
      else
      {
         logWriter.disconnectFromNetworkProcessor();
      }
   }

   private void toggleLoggerServer(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
   {
      if (oldValue == newValue)
         return;

      if (newValue)
      {
         try
         {
            logReader.startServer(portToOpen);
         }
         catch (IOException e)
         {
            e.printStackTrace();
            enableLoggerServerProperty().set(false);
         }
      }
      else
      {
         logReader.stopServer();
      }
   }

   private void togglePause(ObservableValue<? extends Boolean> observable, Boolean oldValue, Boolean newValue)
   {
      if (oldValue == newValue)
         return;

      logReader.pauseReading(newValue);
   }
}
