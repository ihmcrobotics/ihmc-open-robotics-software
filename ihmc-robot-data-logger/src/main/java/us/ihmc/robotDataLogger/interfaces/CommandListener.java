package us.ihmc.robotDataLogger.interfaces;

import us.ihmc.robotDataLogger.websocket.command.DataServerCommand;

public interface CommandListener
{
   /**
    * Gets called when a command is received from the server
    * 
    * @param command
    * @param argument
    */
   public void receivedCommand(DataServerCommand command, int argument);
   
}
