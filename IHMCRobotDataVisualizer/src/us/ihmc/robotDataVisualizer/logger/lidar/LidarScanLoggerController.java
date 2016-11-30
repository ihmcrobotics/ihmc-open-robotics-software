package us.ihmc.robotDataVisualizer.logger.lidar;

import java.io.File;
import java.io.IOException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;

import javafx.beans.InvalidationListener;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.beans.property.StringProperty;
import javafx.beans.value.ObservableValue;
import javafx.stage.FileChooser;
import javafx.stage.FileChooser.ExtensionFilter;
import javafx.stage.Window;
import us.ihmc.communication.configuration.NetworkParameterKeys;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.util.NetworkPorts;

public class LidarScanLoggerController
{
   private static final String DEFAULT_HOST = NetworkParameters.getHost(NetworkParameterKeys.networkManager);

   private final LidarScanLogWriter logWriter = new LidarScanLogWriter();
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

   private StringProperty networkProcessorAddressProperty;

   public StringProperty networkProcessorAddressProperty()
   {
      if (networkProcessorAddressProperty == null)
      {
         networkProcessorAddressProperty = new SimpleStringProperty(this, "networkProcessorAddressProperty", DEFAULT_HOST);
      }
      return networkProcessorAddressProperty;
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
      DateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss_");
      Date time = Calendar.getInstance().getTime();
      return dateFormat.format(time);
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
            logWriter.connectToNetworkProcessor(networkProcessorAddressProperty().get());
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
