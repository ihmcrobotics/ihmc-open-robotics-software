package us.ihmc.robotDataLogger.logger;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

public class LogSettings
{
   public static final LogSettings ATLAS_IAN = new LogSettings(true, "AtlasGUI");
   public static final LogSettings VALKYRIE_IHMC = new LogSettings(true, "ValkyrieIHMCGUI");
   public static final LogSettings VALKYRIE_JSC = new LogSettings(true, "ValkyrieJSCGUI");
   public static final LogSettings STEPPR_IHMC = new LogSettings(true, "StepprIHMCGUI");
   public static final LogSettings SIMULATION = new LogSettings(false, "SimulationGUI");
   public static final LogSettings TEST_LOGGER = new LogSettings(true);
   public static final LogSettings BEHAVIOR = new LogSettings(true);
   public static final LogSettings TOOLBOX = new LogSettings(false);
   public static final LogSettings EXO_X1A = new LogSettings(false);
   public static final LogSettings EXO_HOPPER = new LogSettings(false);
   public static final LogSettings ETHERCAT = new LogSettings(false);
   public static final LogSettings HAND = new LogSettings(false);
   public static final LogSettings MINI_BEAST = new LogSettings(false);
   public static final LogSettings BABY_BEAST = new LogSettings(true);
   public static final LogSettings V2EXO = new LogSettings(true);
   public static final LogSettings MEGABOTS = new LogSettings(true, "MegaBOTSGUI");
   public static final LogSettings FOOTSTEP_PLANNER = new LogSettings(true);

   private final boolean log;
   private final String videoStream;

   public LogSettings(boolean log)
   {
      this(log, null);
   }

   public LogSettings(boolean log, String videoStreamIdentifier)
   {
      this.log = log;
      this.videoStream = videoStreamIdentifier;
   }

   public boolean isLog()
   {
      return log;
   }

   public String getVideoStream()
   {
      return videoStream;
   }
   
   

   @Override
   public String toString()
   {
      return "LogSettings [log=" + log + ", videoStream=" + videoStream + "]";
   }

   public static List<LogSettings> values()
   {
      ArrayList<LogSettings> values = new ArrayList<>();
      Field[] fields = LogSettings.class.getDeclaredFields();
      for (Field f : fields)
      {
         if (Modifier.isStatic(f.getModifiers()) && f.getDeclaringClass() == LogSettings.class)
         {
            try
            {
               values.add((LogSettings) f.get(null));
            }
            catch (IllegalArgumentException | IllegalAccessException e)
            {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
         }
      }
      return values;
   }

   public static void main(String args[])
   {
      System.out.println(values());
   }
}
