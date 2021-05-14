package us.ihmc.robotEnvironmentAwareness.io;

import us.ihmc.log.LogTools;

import java.io.Closeable;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Collections;
import java.util.Enumeration;
import java.util.Properties;
import java.util.TreeSet;

public final class FilePropertyHelper
{
   private final File configurationFile;

   public FilePropertyHelper(String configurationFilePath)
   {
      this(new File(configurationFilePath), 2);
   }

   public FilePropertyHelper(File configurationFile)
   {
      this(configurationFile, 2);
   }

   private FilePropertyHelper(File configurationFile, int stackHeightForPrintout)
   {
      this.configurationFile = ensureFileExists(configurationFile);

      LogTools.debug(stackHeightForPrintout, "Using configuration file: " + configurationFile.toPath().toAbsolutePath().normalize().toString());
   }

   private File ensureFileExists(File file)
   {
      try
      {
         file.getParentFile().mkdirs();
         file.createNewFile();
         return file;
      }
      catch (IOException e)
      {
         System.out.println(file.getAbsolutePath());
         e.printStackTrace();
         return null;
      }
   }

   public void saveProperty(String propertyName, double propertyValue)
   {
      saveProperty(propertyName, Double.toString(propertyValue));
   }

   public void saveProperty(String propertyName, int propertyValue)
   {
      saveProperty(propertyName, Integer.toString(propertyValue));
   }

   public void saveProperty(String propertyName, long propertyValue)
   {
      saveProperty(propertyName, Long.toString(propertyValue));
   }

   public void saveProperty(String propertyName, boolean propertyValue)
   {
      saveProperty(propertyName, Boolean.toString(propertyValue));
   }

   public void saveProperty(String propertyName, String propertyValue)
   {
      if (configurationFile == null)
         return;

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
         if (configurationFile.exists() && configurationFile.isFile())
         {
            fileIn = new FileInputStream(configurationFile);
            properties.load(fileIn);
         }

         properties.setProperty(propertyName, propertyValue);
         fileOut = new FileOutputStream(configurationFile);
         properties.store(fileOut, "");
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when saving property.");
      }
      finally
      {
         closeStreamSilently(fileIn);
         closeStreamSilently(fileOut);
      }
   }

   public Double loadDoubleProperty(String propertyName)
   {
      return loadDoubleProperty(propertyName, null);
   }

   public Double loadDoubleProperty(String propertyName, Double defaultValue)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         return Double.parseDouble(loadedProperty);
      else
         return defaultValue;
   }

   public Boolean loadBooleanProperty(String propertyName)
   {
      return loadBooleanProperty(propertyName, null);
   }

   public Boolean loadBooleanProperty(String propertyName, Boolean defaultValue)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         return Boolean.parseBoolean(loadedProperty);
      else
         return defaultValue;
   }

   public Integer loadIntegerProperty(String propertyName)
   {
      return loadIntegerProperty(propertyName, null);
   }

   public Integer loadIntegerProperty(String propertyName, Integer defaultValue)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         return Integer.parseInt(loadedProperty);
      else
         return defaultValue;
   }

   public Long loadLongProperty(String propertyName)
   {
      return loadLongProperty(propertyName, null);
   }

   public Long loadLongProperty(String propertyName, Long defaultValue)
   {
      String loadedProperty = loadProperty(propertyName);
      if (loadedProperty != null)
         return Long.parseLong(loadedProperty);
      else
         return defaultValue;
   }

   public String loadProperty(String propertyName)
   {
      if (configurationFile == null || !configurationFile.exists() || !configurationFile.isFile())
         return null;

      FileInputStream fileIn = null;
      String propertyValue = null;

      try
      {
         Properties properties = new Properties();

         fileIn = new FileInputStream(configurationFile);
         properties.load(fileIn);
         propertyValue = properties.getProperty(propertyName);
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when loading property.");
      }
      finally
      {
         closeStreamSilently(fileIn);
      }

      return propertyValue;
   }

   private static void closeStreamSilently(Closeable streamToClose)
   {
      try
      {
         if (streamToClose != null)
            streamToClose.close();
      }
      catch (Exception e)
      {
      }
   }
}
