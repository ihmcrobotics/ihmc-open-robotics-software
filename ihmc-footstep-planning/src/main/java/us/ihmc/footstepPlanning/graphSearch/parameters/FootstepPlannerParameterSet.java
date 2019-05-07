package us.ihmc.footstepPlanning.graphSearch.parameters;

import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.File;
import java.io.InputStream;
import java.io.PrintWriter;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.List;
import java.util.Properties;

public class FootstepPlannerParameterSet
{
   private final FootstepPlannerParameterKeys keys;
   private final String saveFileName;

   private final Object[] values;

   public FootstepPlannerParameterSet(FootstepPlannerParameterKeys keys)
   {
      this.keys = keys;
      this.saveFileName = keys.getSaveFileName() + ".ini";

      values = new Object[keys.keys().size()];

      load();
   }

   public double getValue(DoubleFootstepPlannerParameterKey key)
   {
      return (Double) values[key.getIndex()];
   }

   public int getValue(IntegerFootstepPlannerParameterKey key)
   {
      return (Integer) values[key.getIndex()];
   }

   public boolean getValue(BooleanFootstepPlannerParameterKey key)
   {
      return (Boolean) values[key.getIndex()];
   }

   public void setValue(DoubleFootstepPlannerParameterKey key, double value)
   {
      values[key.getIndex()] = value;
   }

   public void setValue(IntegerFootstepPlannerParameterKey key, int value)
   {
      values[key.getIndex()] = value;
   }

   public void setValue(BooleanFootstepPlannerParameterKey key, boolean value)
   {
      values[key.getIndex()] = value;
   }

   public void load()
   {
      ExceptionTools.handle(() ->
      {
         Properties properties = new Properties();
         properties.load(accessStreamForLoading());

         for (FootstepPlannerParameterKey<?> key : keys.keys())
         {
            if (!properties.containsKey(key.getSaveName()))
            {
               throw new RuntimeException(accessUrlForLoading() + " does not contain key: " + key.getSaveName());
            }

            String stringValue = (String) properties.get(key.getSaveName());

            LogTools.info("Loading {}: ({}) {}", key.getSaveName(), stringValue.getClass().getSimpleName(), stringValue);

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
      }, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   public void save()
   {
      ExceptionTools.handle(() ->
      {
         Properties properties = new Properties();

         for (FootstepPlannerParameterKey<?> key : keys.keys())
         {
            properties.setProperty(key.getSaveName(), values[key.getIndex()].toString());
         }

         properties.store(new PrintWriter(findFileForSaving()), LocalDateTime.now().format(DateTimeFormatter.BASIC_ISO_DATE));

      }, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   public static void printInitialSaveFileContents(List<FootstepPlannerParameterKey<?>> keys)
   {
      for (FootstepPlannerParameterKey<?> parameterKey : keys)
      {
         System.out.println(parameterKey.getSaveName() + "=");
      }
   }

   private InputStream accessStreamForLoading()
   {
      return getClass().getResourceAsStream(saveFileName);
   }

   private URL accessUrlForLoading()
   {
      return getClass().getResource(saveFileName);
   }

   private File findFileForSaving()
   {
      return findSaveFileDirectory().resolve(saveFileName).toFile();
   }

   private Path findSaveFileDirectory()
   {
      // find ihmc-open-robotics-software/ihmc-footstep-planning/src/main/java/us/ihmc/footstepPlanning/graphSearch/parameters
      // of just save the file in the working directory

      Path absoluteWorkingDirectory = Paths.get(".").toAbsolutePath().normalize();
      LogTools.info(absoluteWorkingDirectory.toString());

      Path reworkedPath = Paths.get("/").toAbsolutePath().normalize();
      boolean openRoboticsFound = false;
      for (Path path : absoluteWorkingDirectory)
      {
         LogTools.info("Part: {}", path.toString());

         reworkedPath = reworkedPath.resolve(path); // building up the path

         if (path.toString().equals("ihmc-open-robotics-software"))
         {
            openRoboticsFound = true;
            break;
         }
      }

      if (!openRoboticsFound)
      {
         LogTools.warn("Directory \"ihmc-open-robotics-software\" could not be found to save parameters. Using working directory {}",
                       absoluteWorkingDirectory);
         return absoluteWorkingDirectory;
      }

      LogTools.info("Reworked path: {}", reworkedPath);

      Path subPath = Paths.get("ihmc-footstep-planning/src/main/resources/us/ihmc/footstepPlanning/graphSearch/parameters");

      Path finalPath = reworkedPath.resolve(subPath);
      LogTools.info("Final path: {}", finalPath);

      return finalPath;
   }
}
