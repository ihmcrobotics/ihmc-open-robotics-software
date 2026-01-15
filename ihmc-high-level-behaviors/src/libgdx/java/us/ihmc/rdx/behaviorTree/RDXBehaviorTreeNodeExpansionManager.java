package us.ihmc.rdx.behaviorTree;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

/**
 * Maintains a persistent file so the user's preferences for behavior tree expansions are saved.
 */
public class RDXBehaviorTreeNodeExpansionManager
{
   private static final Path EXPANSION_SETTINGS_PATH = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("configurations/BehaviorTreeNodeExpansions.ini");

   private final Map<String, Boolean> expansionStates = new HashMap<>();
   private static int deleteRetries = 0;

   public RDXBehaviorTreeNodeExpansionManager()
   {
      load();
   }

   public boolean isExpanded(String nodeName)
   {
      return expansionStates.getOrDefault(nodeName, true);
   }

   public void setExpanded(String nodeName, boolean expanded)
   {
      expansionStates.put(nodeName, expanded);
      saveAsync();
   }

   public void save()
   {
      try
      {
         File file = EXPANSION_SETTINGS_PATH.toFile();

         file.getParentFile().mkdirs();

         if (!file.exists())
            file.createNewFile();

         Properties properties = new Properties();
         for (Map.Entry<String, Boolean> entry : expansionStates.entrySet())
            properties.setProperty(entry.getKey(), String.valueOf(entry.getValue()));

         try (FileOutputStream output = new FileOutputStream(file))
         {
            properties.store(output, null);
         }
      }
      catch (IOException e)
      {
         LogTools.error(e);
      }
   }

   public void saveAsync()
   {
      ThreadTools.startAsDaemon(this::save, "WriteBehaviorTreeNodeExpansions");
   }

   public void load()
   {
      File file = EXPANSION_SETTINGS_PATH.toFile();

      if (!file.exists())
         save();

      Properties properties = new Properties();

      try
      {
         try (FileInputStream input = new FileInputStream(file))
         {
            properties.load(input);
         }

         expansionStates.clear();
         for (String key : properties.stringPropertyNames())
            expansionStates.put(key, Boolean.parseBoolean(properties.getProperty(key)));
      }
      catch (Exception e)
      {
         // Delete and re-create the file if there was a parsing error
         if (deleteRetries++ > 20)
            return; // Stop gap
         file.delete();
         save();
      }
   }
}
