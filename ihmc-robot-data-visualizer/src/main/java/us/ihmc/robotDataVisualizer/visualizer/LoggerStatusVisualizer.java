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

   private static final String LOGGER_OFFLINE = "[Offline]";
   private static final String CAMERA_OFF = "[Off]";
   private static final String CAMERA_RECODRING = "Recording";


   private final JLabel logger = new JLabel("Logger: ");
   private final JLabel status = new JLabel("");
   private final JLabel camera = new JLabel("Camera: ");
   private final JLabel cameraStatus = new JLabel("");

   private final ScheduledExecutorService offlineExecutor = Executors.newSingleThreadScheduledExecutor();
   private ScheduledFuture<?> offlineExecutorFuture = null;

   public LoggerStatusVisualizer()
   {

      reportLoggerOffline();
   }
   
   public void addToSimulationConstructionSet(SimulationConstructionSet scs)
   {
      scs.addJLabel(logger);
      scs.addJLabel(status);
      scs.addJLabel(camera);
      scs.addJLabel(cameraStatus);
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
            status.setText(String.valueOf(argument) + " seconds");
            cameraStatus.setText(CAMERA_RECODRING);
         });
         startLoggerOfflineTimeout();
      }
      else if (command == DataServerCommand.LOG_ACTIVE)
      {
         SwingUtilities.invokeLater(() -> {

            status.setText(String.valueOf(argument) + " seconds");
            cameraStatus.setText(CAMERA_OFF);
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
         status.setText(LOGGER_OFFLINE);
         cameraStatus.setText(CAMERA_OFF);
      });
   }

}
