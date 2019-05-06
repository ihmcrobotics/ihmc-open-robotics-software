package us.ihmc.footstepPlanning.graphSearch.parameters;

import org.apache.commons.lang3.ArrayUtils;
import org.apache.commons.lang3.ClassUtils;
import org.ini4j.Ini;
import org.ini4j.IniPreferences;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.log.LogTools;

import java.io.File;
import java.io.InputStream;
import java.net.URL;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Properties;
import java.util.TreeMap;
import java.util.prefs.BackingStoreException;

public class FootstepPlannerParameterMap
{
   private final List<FootstepPlannerParameterKey<?>> parameterKeys;
   private final String saveFileName;

   private final TreeMap<Integer, FootstepPlannerParameter<?>> values = new TreeMap<>();

   public FootstepPlannerParameterMap(List<FootstepPlannerParameterKey<?>> parameterKeys, String saveFileName)
   {
      this.parameterKeys = parameterKeys;
      this.saveFileName = saveFileName + ".ini";

      reload();

      for (FootstepPlannerParameterKey<?> key : parameterKeys)
      {
//         values.put(key.getId(), )
      }
   }

   public <T> T getValue(FootstepPlannerParameterKey<T> key)
   {
      FootstepPlannerParameter<T> footstepPlannerParameter = (FootstepPlannerParameter<T>) values.get(key.getId());
      return footstepPlannerParameter.getValue();
   }

   public void reload()
   {
      ExceptionTools.handle(() ->
      {
//         Ini ini = new Ini(accessStreamForLoading());
//         IniPreferences preferences = new IniPreferences(ini);

         Properties properties = new Properties();
         properties.load(accessStreamForLoading());

//         properties.containsKey()

         for (FootstepPlannerParameterKey<?> key : parameterKeys)
         {
            if (!properties.containsKey(key.getSaveName()))
            {
               throw new RuntimeException(accessUrlForLoading() + " does not contain key: " + key.getSaveName());
            }

            Object stringValue = properties.get(key.getSaveName());

            LogTools.info("{}: ({}) {}", key.getSaveName(), stringValue.getClass().getSimpleName(), stringValue);


//            ClassUtils.isPrimitiveOrWrapper()
//
//
//            if (Double.valueOf(stringValue))

//            values.put(key.getId(), );
         }

//         for (String key : preferences.keys())
//         {
//            System.out.println(key);
//
//
//
//         }

      }, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   private boolean preferencesContainKey(IniPreferences preferences, FootstepPlannerParameterKey<?> key) throws BackingStoreException
   {
      for (String foundKey : preferences.keys())
      {
         LogTools.info("{}:{}", foundKey, key.getSaveName());
         if (foundKey.equals(key.getSaveName()))
         {
            return true;
         }
      }

      return false;
   }

   public void save()
   {
      ExceptionTools.handle(() ->
      {
         Ini ini = new Ini(accessStreamForLoading());
         IniPreferences preferences = new IniPreferences(ini);

         for (FootstepPlannerParameterKey<?> parameterKey : parameterKeys)
         {
            preferences.put(parameterKey.getSaveName(), "");
         }

         ini.store();
      }, DefaultExceptionHandler.PRINT_STACKTRACE);
   }

   public void printInitializedSaveFile()
   {
      for (FootstepPlannerParameterKey<?> parameterKey : parameterKeys)
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

      Path subPath = Paths.get("ihmc-footstep-planning/src/main/java/us/ihmc/footstepPlanning/graphSearch/parameters");

      Path finalPath = reworkedPath.resolve(subPath);
      LogTools.info("Final path: {}", finalPath);

      return finalPath;
   }
}
