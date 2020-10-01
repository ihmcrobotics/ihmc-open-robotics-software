package us.ihmc.robotics.saveableModule;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Properties;
import java.util.TreeSet;

public abstract class SaveableModule<T extends SaveableModuleState>
{
   private File file;
   private T state;

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

   public void load(String propertyName)
   {
      if (file == null|| !file.exists() || !file.isFile())
         throw new IllegalArgumentException("File has not been set.");
      if (state == null)
         throw new IllegalArgumentException("State has not been set.");

      FileInputStream fileIn = null;
      String propertyValue = null;

      try
      {
         Properties properties = new Properties();

         fileIn = new FileInputStream(file);
         properties.load(fileIn);
         propertyValue = properties.getProperty(propertyName);
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when loading module.");
      }
      finally
      {
         SaveableModuleTools.closeStreamSilently(fileIn);
      }

      state.loadValues(propertyValue);
   }


   public void save(String propertyName)
   {
      if (file == null)
         throw new IllegalArgumentException("File has not been set.");
      if (state == null)
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

         properties.setProperty(propertyName, state.toString());
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
