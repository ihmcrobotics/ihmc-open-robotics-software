package us.ihmc.robotDataVisualizer.visualizer;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import javax.swing.JLabel;
import javax.swing.SwingUtilities;

import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * Simple JPanel to visualize the status of the logger
 * 
 * @author Jesper Smith
 *
 */
public class LoggerStatusVisualizer
{
   private final static int LOGGER_STATUS_TIMEOUT_SECONDS = 6;
   
   
   private final Object schedulerLock = new Object();

   private static final String LOGGER_OFFLINE = "<html>Log duration: <br>[Log Offline]</html>";
   private static final String LOGGING_FOR = "<html>Log duration: <br>";
   private static final String LOGGER_SECONDS = " s</html>";
   
   private static final String CAMERA_OFF = "<html>Camera: <br>[Off]</html>";
   private static final String CAMERA_RECORDING = "<html>Camera: <br>[Off]</html>";


   private final JLabel logger = new JLabel(LOGGER_OFFLINE);
   private final JLabel camera = new JLabel(CAMERA_OFF);

   private final ScheduledExecutorService offlineExecutor = Executors.newSingleThreadScheduledExecutor();
   private ScheduledFuture<?> offlineExecutorFuture = null;

   public LoggerStatusVisualizer()
   {

      reportLoggerOffline();
   }
   
   public void addToSimulationConstructionSet(SimulationConstructionSet scs)
   {
      scs.addJLabel(logger);
      scs.addJLabel(camera);
   }
   

   /**
    * Receive the logger status from the command server
    * 
    * Will only act on LOG_ACTIVE_WITH_CAMERA or LOG_ACTIVE commands
    * 
    * @param command
    * @param argument
    */
   public void updateStatus(DataServerCommand command, int argument)
   {
      if (command == DataServerCommand.LOG_ACTIVE_WITH_CAMERA)
      {
         SwingUtilities.invokeLater(() -> {
            logger.setText(LOGGING_FOR + String.valueOf(argument) + LOGGER_SECONDS);
            camera.setText(CAMERA_RECORDING);
         });
         startLoggerOfflineTimeout();
      }
      else if (command == DataServerCommand.LOG_ACTIVE)
      {
         SwingUtilities.invokeLater(() -> {

            logger.setText(LOGGING_FOR + String.valueOf(argument) + LOGGER_SECONDS);
            camera.setText(CAMERA_OFF);
         });
         startLoggerOfflineTimeout();
      }
   }

   private void startLoggerOfflineTimeout()
   {
      synchronized (schedulerLock)
      {
         if(offlineExecutorFuture != null)
         {
            offlineExecutorFuture.cancel(false);
         }
         
         offlineExecutorFuture = offlineExecutor.schedule(() -> reportLoggerOffline(), LOGGER_STATUS_TIMEOUT_SECONDS, TimeUnit.SECONDS);
      }
   }

   private void reportLoggerOffline()
   {
      SwingUtilities.invokeLater(() -> {
         logger.setText(LOGGER_OFFLINE);
         camera.setText(CAMERA_OFF);
      });
   }

}
