package us.ihmc.tools.io.logging;

import java.util.logging.ConsoleHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

public class LogTools
{
   public static void setGlobalLogLevel(Level level)
   {
      Logger.getLogger("").setLevel(level);
      ConsoleHandler handler = new ConsoleHandler();
      handler.setLevel(level);
      Logger.getLogger("").addHandler(handler);
   }

   public static Level getGlobalLogLevel()
   {
      return Logger.getLogger("").getLevel();
   }
}
