package us.ihmc.robotEnvironmentAwareness.io;

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

   public FilePropertyHelper(File configurationFile)
   {
      this.configurationFile = configurationFile;
   }

   public void saveProperty(String propertyName, double propertyValue)
   {
      saveProperty(propertyName, Double.toString(propertyValue));
   }

   public void saveProperty(String propertyName, int propertyValue)
   {
      saveProperty(propertyName, Integer.toString(propertyValue));
   }

   public void saveProperty(String propertyName, boolean propertyValue)
   {
      saveProperty(propertyName, Boolean.toString(propertyValue));
   }

   public void saveProperty(String propertyName, String propertyValue)
   {
      FileOutputStream fileOut = null;
      FileInputStream fileIn = null;

      try
      {
         Properties properties = new Properties()
         {
            private static final long serialVersionUID = -8814683165980261816L;

            @Override
            public synchronized Enumeration<Object> keys()
            {
               return Collections.enumeration(new TreeSet<Object>(super.keySet()));
            }
         };


         if (configurationFile.exists() && configurationFile.isFile())
         {
            fileIn = new FileInputStream(configurationFile);
            properties.load(fileIn);
         }

         properties.setProperty(propertyName, propertyValue);
         fileOut = new FileOutputStream(configurationFile);
         properties.store(fileOut, "");
         fileOut.close();
      }
      catch (Exception ex)
      {
         throw new RuntimeException("Problem when saving property.");
      }
      finally
      {
         try
         {
            if (fileIn != null)
               fileIn.close();
         }
         catch (Exception e)
         {
         }

         try
         {
            fileOut.close();
         }
         catch (IOException e)
         {
         }
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

   public String loadProperty(String propertyName)
   {
      FileInputStream fileIn = null;
      String propertyValue = null;

      if (!configurationFile.exists() || !configurationFile.isFile())
         return null;

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
         try
         {
            fileIn.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      return propertyValue;
   }
}
