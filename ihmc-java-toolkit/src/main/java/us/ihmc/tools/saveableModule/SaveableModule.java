package us.ihmc.tools.saveableModule;

import us.ihmc.tools.io.WorkspacePathTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import javax.swing.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Properties;
import java.util.TreeSet;

public abstract class SaveableModule<T extends SaveableModuleState>
{
   private static final Path rootPath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");

   private File file;
   private T state;

   private final YoBoolean saveStateFileTrigger;

   private final Class<T> clazz;
   private final String moduleName;

   public SaveableModule(Class<T> clazz, YoRegistry registry)
   {
      this(clazz, clazz.getName(), registry);
   }

   public SaveableModule(Class<T> clazz, String moduleName, YoRegistry registry)
   {
      this.clazz = clazz;
      this.moduleName = moduleName;
      saveStateFileTrigger = new YoBoolean(moduleName + "SaveStateFileTrigger", registry);
      saveStateFileTrigger.addListener((v) ->
                                       {
                                          if (saveStateFileTrigger.getBooleanValue())
                                          {
                                             saveStateFileTrigger.set(false, false);
                                             save();
                                          }
                                       });
   }


   public abstract void compute(T state);

   public void setFileToSave(File file)
   {
      this.file = SaveableModuleTools.ensureFileExists(file);
   }

   public void setFilePathToSave(String filePath)
   {
      setFileToSave(new File(filePath));
   }

   public void setSaveableModuleState(T state)
   {
      this.state = state;
   }

   public void load()
   {
      File file = this.file;
      if (file == null)
      {
         JFileChooser fileChooser = new JFileChooser();
         File logDirectory = new File(getDefaultLogsDirectory());
         fileChooser.setCurrentDirectory(logDirectory);
         fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
         int chooserState = fileChooser.showOpenDialog(null);

         if (chooserState != JFileChooser.APPROVE_OPTION)
            return;

         file = fileChooser.getSelectedFile();
      }

      load(file, moduleName, state);
   }

   // FIXME fix the saving of the log here.
   public void save()
   {
      File file = this.file;
      if (file == null)
      {
         JFileChooser fileChooser = new JFileChooser();
         File logDirectory = new File(getDefaultLogsDirectory());
         fileChooser.setCurrentDirectory(logDirectory);
         fileChooser.setFileSelectionMode(JFileChooser.FILES_ONLY);
         int chooserState = fileChooser.showOpenDialog(null);

         if (chooserState != JFileChooser.APPROVE_OPTION)
            return;

         // TODO get the default log directory from the class name. enter the stupid file name.

         file = fileChooser.getSelectedFile();
      }

      save(file, moduleName, state);
   }

   private String getDefaultLogsDirectory()
   {
      // check this package name thing.
      Path path = Paths.get(rootPath.toString(), clazz.getPackage().getName());
      return path.toString();
   }

   public static void load(File file, String moduleName, SaveableModuleState stateToPack)
   {
      if (file == null|| !file.exists() || !file.isFile())
         throw new IllegalArgumentException("File has not been set.");
      if (stateToPack == null)
         throw new IllegalArgumentException("State has not been set.");

      FileInputStream fileIn = null;
      String propertyValue = null;

      try
      {
         Properties properties = new Properties();

         fileIn = new FileInputStream(file);
         properties.load(fileIn);
         propertyValue = properties.getProperty(moduleName);
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when loading module.");
      }
      finally
      {
         SaveableModuleTools.closeStreamSilently(fileIn);
      }

      stateToPack.loadValues(propertyValue);
   }

   public static void save(File file, String moduleName, SaveableModuleState stateToSave)
   {
      if (file == null)
         throw new IllegalArgumentException("File has not been set.");
      if (stateToSave == null)
         throw new IllegalArgumentException("State has not been set.");

      Properties properties = new Properties()
      {
         private static final long serialVersionUID = -8814683165980261816L;

         @Override
         public synchronized Enumeration<Object> keys()
         {
            return Collections.enumeration(new TreeSet<Object>(super.keySet()));
         }
      };

      FileInputStream fileIn = null;
      FileOutputStream fileOut = null;

      try
      {
         if (file.exists() && file.isFile())
         {
            fileIn = new FileInputStream(file);
            properties.load(fileIn);
         }

         properties.setProperty(moduleName, stateToSave.toString());
         fileOut = new FileOutputStream(file);
         properties.store(fileOut, "");
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when saving module.");
      }
      finally
      {
         SaveableModuleTools.closeStreamSilently(fileIn);
         SaveableModuleTools.closeStreamSilently(fileOut);
      }
   }


}
