package us.ihmc.avatar.joystickBasedJavaFXController;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Scanner;
import java.util.function.Function;
import java.util.regex.Pattern;
import java.util.stream.Collectors;

import us.ihmc.log.LogTools;

public class UserProfileManager<T>
{
   public static final String fileExtension = ".ini";
   public static final String defaultWorkingDirectoryPath = System.getProperty("user.home") + "/.ihmc/joystick_step_app/";
   public final static String userProfilesFilename = "profiles" + fileExtension;

   private final T defaultParameters;
   private final File workingDirectory;
   private final PropertyMapParser<T> parser;
   private final PropertyMapExporter<T> exporter;

   public UserProfileManager(String workingDirectoryPath, T defaultParameters, PropertyMapParser<T> parser, PropertyMapExporter<T> exporter)
   {
      this.defaultParameters = defaultParameters;
      this.parser = parser;
      this.exporter = exporter;

      if (workingDirectoryPath == null)
         workingDirectoryPath = UserProfileManager.defaultWorkingDirectoryPath;

      this.workingDirectory = new File(workingDirectoryPath);
      workingDirectory.mkdirs();
   }

   public List<String> getUserProfileNames()
   {
      return readUserProfileNames(workingDirectory);
   }

   public T loadProfile(String profileName)
   {
      return loadProfileParameters(workingDirectory, profileName, defaultParameters, parser, exporter);
   }

   public void saveProfile(String profileName, T parameters)
   {
      saveProfileParameters(workingDirectory, profileName, parameters, exporter);
   }

   public void close(List<String> userProfileNames)
   {
      saveUserProfileNames(workingDirectory, userProfileNames);
      List<File> parameterFiles = Arrays.asList(workingDirectory.listFiles(filename -> !filename.getName().equals(userProfilesFilename)));
      for (File parameterFile : parameterFiles)
      {
         String profileName = parameterFile.getName().replaceAll(fileExtension, "");

         if (!userProfileNames.contains(profileName))
            parameterFile.deleteOnExit();
      }
   }

   private static List<String> readUserProfileNames(File workingDirectory)
   {
      File userProfileNamesFile = new File(workingDirectory, UserProfileManager.userProfilesFilename);
      List<String> userProfileNameList = new ArrayList<>();
      Scanner scanner;
      try
      {
         scanner = new Scanner(userProfileNamesFile);
      }
      catch (FileNotFoundException e)
      {
         LogTools.error("User profiles file not found");
         return userProfileNameList;
      }

      scanner.useDelimiter(Pattern.compile("\n"));

      while (scanner.hasNext())
      {
         String next = scanner.next();
         next = next.replaceAll(" ", "").replaceAll("\r", "");
         if (!next.isEmpty())
            userProfileNameList.add(next);
      }

      scanner.close();
      return userProfileNameList;
   }

   private static void saveUserProfileNames(File workingDirectory, List<String> userProfileNames)
   {
      File userProfileNamesFile = new File(workingDirectory, UserProfileManager.userProfilesFilename);
      FileWriter fileWriter = null;

      try
      {
         fileWriter = new FileWriter(userProfileNamesFile);
         for (int i = 0; i < userProfileNames.size(); i++)
         {
            if (i > 0)
               fileWriter.write("\n");
            String userProfileName = userProfileNames.get(i);
            fileWriter.write(userProfileName);
         }
      }
      catch (IOException e)
      {
         throw new RuntimeException("Encountered problem saving profile names.", e);
      } finally
      {
         try
         {
            if (fileWriter != null)
               fileWriter.close();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Encountered problem saving profile names.", e);
         }
      }
   }

   private static <T> T loadProfileParameters(File workingDirectory, String userProfileName, T defaultParameters, PropertyMapParser<T> parser,
                                              PropertyMapExporter<T> exporter)
   {
      if (userProfileName == null)
         return defaultParameters;

      T loadedParameters;

      File userParameterFile = createParameterFile(workingDirectory, userProfileName);

      if (!userParameterFile.exists())
      {
         loadedParameters = defaultParameters;
         saveProfileParameters(workingDirectory, userProfileName, defaultParameters, exporter);
      }
      else
      {
         try
         {
            FileInputStream fileInputStream = new FileInputStream(userParameterFile);
            Properties properties = new Properties();
            properties.load(fileInputStream);
            Map<String, String> propertyMap = properties.stringPropertyNames().stream()
                                                        .collect(Collectors.toMap(Function.identity(), key -> properties.getProperty(key)));
            loadedParameters = parser.parseFromPropertyMap(propertyMap, defaultParameters);
            fileInputStream.close();
         }
         catch (FileNotFoundException e)
         {
            throw new RuntimeException("Should not get there as the file existence has been verified already.", e);
         }
         catch (IOException e)
         {
            throw new RuntimeException("Encountered problem loading " + userProfileName, e);
         }
      }
      return loadedParameters;
   }

   private static <T> void saveProfileParameters(File workingDirectory, String userProfileName, T parameters, PropertyMapExporter<T> exporter)
   {
      if (userProfileName == null || parameters == null)
         return;

      File userParameterFile = createParameterFile(workingDirectory, userProfileName);
      if (!userParameterFile.exists())
      {
         try
         {
            userParameterFile.createNewFile();
         }
         catch (IOException e)
         {
            throw new RuntimeException("Encountered problem saving " + userProfileName, e);
         }
      }

      FileWriter fileWriter = null;
      try
      {
         fileWriter = new FileWriter(userParameterFile);
         Properties properties = new Properties();
         exporter.exportAsProPertyMap(parameters).entrySet().forEach(entry -> properties.setProperty(entry.getKey(), entry.getValue()));
         properties.store(fileWriter, "Paramters for the IHMC joystick stepping application");
      }
      catch (IOException e)
      {
         throw new RuntimeException("Encountered problem saving " + userProfileName, e);
      } finally
      {
         if (fileWriter != null)
         {
            try
            {
               fileWriter.close();
            }
            catch (IOException e)
            {
               throw new RuntimeException("Encountered problem saving " + userProfileName, e);
            }
         }
      }
   }

   private static File createParameterFile(File workingDirectory, String userProfileName)
   {
      return new File(workingDirectory, userProfileName + fileExtension);
   }

   public static interface PropertyMapParser<T>
   {
      T parseFromPropertyMap(Map<String, String> properties, T defaultParameters);
   }

   public static interface PropertyMapExporter<T>
   {
      Map<String, String> exportAsProPertyMap(T parametersToExport);
   }
}
