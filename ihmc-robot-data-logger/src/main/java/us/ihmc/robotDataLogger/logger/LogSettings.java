package us.ihmc.robotDataLogger.logger;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

/**
 * Replaced by DataServerSettings
 * 
 * Do not add static LogSettings in this file anymore.
 * 
 * You should instead pass in a "new DataServerSettings()" to the YoVariableServer with the desired variables
 *  
 * @author Jesper Smith
 *
 */
@Deprecated
public class LogSettings extends DataServerSettings
{
   @Deprecated
   public static final LogSettings ATLAS_IAN = new LogSettings(true, "AtlasGUI");
   @Deprecated
   public static final LogSettings VALKYRIE_IHMC = new LogSettings(true, "ValkyrieIHMCGUI");
   @Deprecated
   public static final LogSettings VALKYRIE_JSC = new LogSettings(true, "ValkyrieJSCGUI");
   @Deprecated
   public static final LogSettings STEPPR_IHMC = new LogSettings(true, "StepprIHMCGUI");
   @Deprecated
   public static final LogSettings SIMULATION = new LogSettings(false, "SimulationGUI");
   @Deprecated
   public static final LogSettings TEST_LOGGER = new LogSettings(true);
   @Deprecated
   public static final LogSettings BEHAVIOR = new LogSettings(false);
   @Deprecated
   public static final LogSettings TOOLBOX = new LogSettings(false);
   @Deprecated
   public static final LogSettings EXO_X1A = new LogSettings(false);
   @Deprecated
   public static final LogSettings EXO_HOPPER = new LogSettings(false);
   @Deprecated
   public static final LogSettings ETHERCAT = new LogSettings(false);
   @Deprecated
   public static final LogSettings HAND = new LogSettings(false);
   @Deprecated
   public static final LogSettings MINI_BEAST = new LogSettings(false);
   @Deprecated
   public static final LogSettings BABY_BEAST = new LogSettings(true);
   @Deprecated
   public static final LogSettings V2EXO = new LogSettings(true);
   @Deprecated
   public static final LogSettings MEGABOTS = new LogSettings(true, "MegaBOTSGUI");
   @Deprecated
   public static final LogSettings FOOTSTEP_PLANNER = new LogSettings(false);
   @Deprecated
   public static final LogSettings LLAMA = new LogSettings(true, "LlamaRemoteUIStream");
   @Deprecated
   public static final LogSettings THOR = new LogSettings(false);

   // Do not add keys here. Put them in the project or just pass in new DataServerSettings(true/false) to the YoVariableServer.

   @Deprecated
   private LogSettings(boolean log)
   {
      super(log);
   }

   @Deprecated
   private LogSettings(boolean log, String videoStreamIdentifier)
   {
      super(log, videoStreamIdentifier);
   }
   
   @Deprecated
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
