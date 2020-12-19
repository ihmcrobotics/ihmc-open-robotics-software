package us.ihmc.gdx;

import com.badlogic.gdx.Application;
import com.badlogic.gdx.Gdx;
import org.apache.logging.log4j.Level;
import us.ihmc.log.LogTools;

public class GDXTools
{
   public static void syncLogLevelWithLogTools()
   {
      Level log4jLevel = LogTools.getLevel();
      int gdxLogLevel = 2;
      switch (log4jLevel.getStandardLevel())
      {
         case OFF:
            gdxLogLevel = Application.LOG_NONE;
            break;
         case FATAL:
         case ERROR:
            gdxLogLevel = Application.LOG_ERROR;
            break;
         case WARN:
         case INFO:
            gdxLogLevel = Application.LOG_INFO;
            break;
         case DEBUG:
         case TRACE:
            gdxLogLevel = Application.LOG_DEBUG;
            break;
      }
      Gdx.app.setLogLevel(gdxLogLevel);
   }
}
