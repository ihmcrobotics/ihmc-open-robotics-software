package us.ihmc.tools.property;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.commons.nio.WriteOption;
import us.ihmc.log.LogTools;

import java.io.InputStream;
import java.io.PrintWriter;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

/**
 * Provides a load/saveable property set accessed by strongly typed static keys.
 *
 * The property INI file is saved to the classpath by file and loaded from the classpath by resource.
 *
 * Some of the benefits of this framework:
 * - Keys are created with title cased names available for GUI fields
 * - No YoVariableServer required
 * - INI file can be placed in higher level projects to override the defaults
 */
public class StoredPropertySet implements StoredPropertySetReadOnly
{
   private final StoredPropertyKeyList keys;
   private final String saveFileName;

   private final Object[] values;
   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;

   public StoredPropertySet(StoredPropertyKeyList keys,
                            Class<?> classForLoading,
                            String directoryNameToAssumePresent,
                            String subsequentPathToResourceFolder)
   {
      this.keys = keys;
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;

      this.saveFileName = keys.getSaveFileName() + ".ini";

      values = new Object[keys.keys().size()];
   }

   @Override
   public double get(DoubleStoredPropertyKey key)
   {
      return (Double) values[key.getIndex()];
   }

   @Override
   public int get(IntegerStoredPropertyKey key)
   {
      return (Integer) values[key.getIndex()];
   }

   @Override
   public boolean get(BooleanStoredPropertyKey key)
   {
      return (Boolean) values[key.getIndex()];
   }

   public void set(DoubleStoredPropertyKey key, double value)
   {
      values[key.getIndex()] = value;
   }

   public void set(IntegerStoredPropertyKey key, int value)
   {
      values[key.getIndex()] = value;
   }

   public void set(BooleanStoredPropertyKey key, boolean value)
   {
      values[key.getIndex()] = value;
   }

   @Override
   public List<Object> getAll()
   {
      return Arrays.asList(values);
   }

   public void setAll(List<Object> newValues)
   {
      for (int i = 0; i < values.length; i++)
      {
         values[i] = newValues.get(i);
      }
   }

   public void load()
   {
      ExceptionTools.handle(() ->
      {
         Properties properties = new Properties();
         InputStream streamForLoading = accessStreamForLoading();

         if (streamForLoading == null)
         {
            LogTools.warn("Parameter file {} could not be found. Values will be null.", saveFileName);
         }
         else
         {
            LogTools.info("Loading parameters from {}", saveFileName);
            properties.load(streamForLoading);

            for (StoredPropertyKey<?> key : keys.keys())
            {
               if (!properties.containsKey(key.getCamelCasedName()))
               {
                  throw new RuntimeException(accessUrlForLoading() + " does not contain key: " + key.getCamelCasedName());
               }

               String stringValue = (String) properties.get(key.getCamelCasedName());

               if (stringValue.equals("null"))
               {
                  LogTools.warn("{} is being loaded as null. Please set it in {}", key.getCamelCasedName(), saveFileName);
               }
               else
               {
                  if (key.getType().equals(Double.class))
                  {
                     values[key.getIndex()] = Double.valueOf(stringValue);
                  }
                  else if (key.getType().equals(Integer.class))
                  {
                     values[key.getIndex()] = Integer.valueOf(stringValue);
                  }
                  else if (key.getType().equals(Boolean.class))
                  {
                     values[key.getIndex()] = Boolean.valueOf(stringValue);
                  }
                  else
                  {
                     throw new RuntimeException("Please implement String deserialization for type: " + key.getType());
                  }
               }

            }
         }
      }, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   public void save()
   {
      ExceptionTools.handle(() ->
      {
         Properties properties = new Properties();

         for (StoredPropertyKey<?> key : keys.keys())
         {
            if (values[key.getIndex()] == null)
            {
               properties.setProperty(key.getCamelCasedName(), "null");
            }
            else
            {
               properties.setProperty(key.getCamelCasedName(), values[key.getIndex()].toString());
            }
         }

         Path fileForSaving = findFileForSaving();
         properties.store(new PrintWriter(fileForSaving.toFile()), LocalDateTime.now().format(DateTimeFormatter.BASIC_ISO_DATE));

         convertLineEndingsToUnix(fileForSaving);
      }, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   private void convertLineEndingsToUnix(Path fileForSaving)
   {
      List<String> lines = FileTools.readAllLines(fileForSaving, DefaultExceptionHandler.PRINT_STACKTRACE);
      PrintWriter printer = FileTools.newPrintWriter(fileForSaving, WriteOption.TRUNCATE, DefaultExceptionHandler.PRINT_STACKTRACE);
      lines.forEach(line -> printer.print(line + "\n"));
      printer.close();
   }

   public static void printInitialSaveFileContents(List<StoredPropertyKey<?>> keys)
   {
      for (StoredPropertyKey<?> parameterKey : keys)
      {
         System.out.println(parameterKey.getCamelCasedName() + "=");
      }
   }

   private InputStream accessStreamForLoading()
   {
      return classForLoading.getResourceAsStream(saveFileName);
   }

   private URL accessUrlForLoading()
   {
      return classForLoading.getResource(saveFileName);
   }

   private Path findFileForSaving()
   {
      return findSaveFileDirectory().resolve(saveFileName);
   }

   private Path findSaveFileDirectory()
   {
      // find, for example, ihmc-open-robotics-software/ihmc-footstep-planning/src/main/java/us/ihmc/footstepPlanning/graphSearch/parameters
      // of just save the file in the working directory

      Path absoluteWorkingDirectory = Paths.get(".").toAbsolutePath().normalize();

      Path reworkedPath = Paths.get("/").toAbsolutePath().normalize(); // start with system root
      boolean directoryFound = false;
      for (Path path : absoluteWorkingDirectory)
      {
         reworkedPath = reworkedPath.resolve(path); // building up the path

         if (path.toString().equals(directoryNameToAssumePresent))
         {
            directoryFound = true;
            break;
         }
      }

      if (!directoryFound && Files.exists(reworkedPath.resolve(directoryNameToAssumePresent))) // working directory is workspace
      {
         reworkedPath = reworkedPath.resolve(directoryNameToAssumePresent);
         directoryFound = true;
      }

      if (!directoryFound)
      {
         LogTools.warn("Directory {} could not be found to save parameters. Using working directory {}. Reworked path: {}",
                       directoryNameToAssumePresent,
                       absoluteWorkingDirectory,
                       reworkedPath);
         return absoluteWorkingDirectory;
      }

      String s = classForLoading.getPackage().toString();
      LogTools.debug(s);
      String packagePath = s.split(" ")[1].replaceAll("\\.", "/");
      LogTools.debug(packagePath);

      Path subPath = Paths.get(subsequentPathToResourceFolder, packagePath);

      Path finalPath = reworkedPath.resolve(subPath);

      FileTools.ensureDirectoryExists(finalPath, DefaultExceptionHandler.PRINT_STACKTRACE);

      return finalPath;
   }
}