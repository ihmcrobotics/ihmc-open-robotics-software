package us.ihmc.robotDataVisualizer.visualizer;

import javax.swing.BoxLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

/**
 * Simple JPanel to visualize the status of the logger
 * 
 * @author Jesper Smith
 *
 */
public class LoggerStatusVisualizer extends JPanel
{
   private static final String LOGGER_OFFLINE = "[Offline]";
   private static final String CAMERA_OFF = "[Off]";
   
   
   
   private static final long serialVersionUID = -3518142795365476996L;
   
   private final JLabel logger = new JLabel("Logger: ");
   private final JLabel status = new JLabel("");
   private final JLabel camera = new JLabel("Camera: ");
   private final JLabel cameraStatus = new JLabel("");
   
   
   public LoggerStatusVisualizer()
   {
      setLayout(new BoxLayout(this, BoxLayout.X_AXIS));  
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
      if(command == DataServerCommand.LOG_ACTIVE_WITH_CAMERA)
      {
         
      }
      else if (command == DataServerCommand.LOG_ACTIVE)
      {
         
      }
         
   }

}
