package us.ihmc.rdx.behaviorTree;

import us.ihmc.log.LogTools;
import us.ihmc.rdx.behaviorTree.actions.RDXActionProgressWidgetsManager.Type;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Properties;
import java.util.concurrent.CompletableFuture;

public class RDXBehaviorTreeSettings
{
   private static final Path PATH = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("RDXBehaviorTreeSettings.ini");
   private static int deleteRetries = 0;

   /** 60% seems to be the desirable ratio for the visible area of the tree view vs the settings area */
   private float treeExplorerHeightPercentage = 0.6f;
   private Type progressWidgetsType = Type.PROGRESS_BARS;

   public float getTreeExplorerHeightPercentage()
   {
      return treeExplorerHeightPercentage;
   }

   public void setTreeExplorerHeightPercentage(float treeExplorerHeightPercentage)
   {
      this.treeExplorerHeightPercentage = treeExplorerHeightPercentage;
      saveAsync();
   }

   public Type getProgressWidgetsType()
   {
      return progressWidgetsType;
   }

   public void setProgressWidgetsType(Type progressWidgetsType)
   {
      this.progressWidgetsType = progressWidgetsType;
      saveAsync();
   }

   public RDXBehaviorTreeSettings()
   {
      try
      {
         File file = PATH.toFile();

         if (!file.exists())
         {
            save();
         }

         Properties properties = new Properties();

         try (FileInputStream input = new FileInputStream(file))
         {
            properties.load(input);
         }

         try
         {
            treeExplorerHeightPercentage = Float.parseFloat(properties.getProperty("treeExplorerHeightPercentage"));
            progressWidgetsType = Type.valueOf(properties.getProperty("progressWidgetsType"));
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
      catch (IOException e)
      {
         LogTools.error("Unable to load RDXBehaviorTreeSettings.ini", e);
      }
   }

   public void save() throws IOException
   {
      File file = PATH.toFile();

      file.getParentFile().mkdirs();

      if (!file.exists())
         file.createNewFile();

      Properties properties = new Properties();
      properties.setProperty("treeExplorerHeightPercentage", String.valueOf(treeExplorerHeightPercentage));
      properties.setProperty("progressWidgetsType", progressWidgetsType.name());

      try (FileOutputStream output = new FileOutputStream(file))
      {
         properties.store(output, null);
      }
   }

   private void saveAsync()
   {
      CompletableFuture.runAsync(() ->
      {
         try
         {
            save();
         }
         catch (IOException e)
         {
            LogTools.info(e);
         }
      });
   }
}
