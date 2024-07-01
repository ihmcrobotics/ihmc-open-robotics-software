package us.ihmc.tools.io;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.log.LogTools;

import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.Consumer;

public class JSONFileTools
{
   public static void loadWithClasspathDefault(Path settingsPath,
                                               Class<?> classForLoading,
                                               String directoryNameToAssumePresent,
                                               String subsequentPathToResourceFolder,
                                               String resourcePackage,
                                               Consumer<JsonNode> jsonNodeConsumer)
   {
      InputStream settingsStream;
      if (!Files.exists(settingsPath))
      {
         String resourcePathString = Paths.get(resourcePackage).resolve(settingsPath.getFileName()).toString();
         settingsStream = classForLoading.getResourceAsStream(resourcePathString);
         if (settingsStream == null)
         {
            String fullResourcePathString = WorkspacePathTools.findPathToResource(directoryNameToAssumePresent,
                                                                                  subsequentPathToResourceFolder,
                                                                                  resourcePathString).resolve(settingsPath.getFileName()).toString();
            LogTools.warn("{} not found. Please save defaults to {}. Not loading anything.", settingsPath.toString(), fullResourcePathString);
            return;
         }
         else
         {
            LogTools.info("{} not found. Loading defaults from {}", settingsPath.toString(), resourcePathString);
         }
      }
      else
      {
         LogTools.info("Loading {}", settingsPath.toString());
         settingsStream = FileTools.newFileDataInputStream(settingsPath, DefaultExceptionHandler.PRINT_STACKTRACE);
      }

      try (InputStream closableStream = settingsStream)
      {
         loadInternal(closableStream, jsonNodeConsumer);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void loadUserWithClasspathDefaultFallback(HybridResourceFile hybridFile, Consumer<JsonNode> jsonNodeConsumer)
   {
      InputStream settingsStream;
      if (!Files.exists(hybridFile.getExternalFile()))
      {
         settingsStream = hybridFile.getClasspathResourceAsStream();
         if (settingsStream == null)
         {
            LogTools.warn("Defaults not found. Please save to {}. Not loading anything.", hybridFile.getPathForResourceLoadingPathFiltered());
            return;
         }
         else
         {
            LogTools.info("{} not found. Loading defaults from {}",
                          hybridFile.getExternalFile().toString(),
                          hybridFile.getPathForResourceLoadingPathFiltered());
         }
      }
      else
      {
         LogTools.info("Loading {}", hybridFile.getExternalFile().toString());
         settingsStream = FileTools.newFileDataInputStream(hybridFile.getExternalFile(), DefaultExceptionHandler.PRINT_STACKTRACE);
      }

      try (InputStream closableStream = settingsStream)
      {
         loadInternal(closableStream, jsonNodeConsumer);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void load(WorkspaceResourceFile file, Consumer<JsonNode> jsonNodeConsumer)
   {
      load(file.getClasspathResourceAsStream(), jsonNodeConsumer);
   }

   public static void loadFromWorkspace(String directoryNameToAssumePresent,
                                        String subsequentPathToResourceFolder,
                                        String resourcePathString,
                                        Consumer<JsonNode> jsonNodeConsumer)
   {
      load(WorkspacePathTools.findPathToResource(directoryNameToAssumePresent, subsequentPathToResourceFolder, resourcePathString), jsonNodeConsumer);
   }

   public static boolean loadFromClasspath(Class<?> classForLoading,
                                           String resourcePathString,
                                           Consumer<JsonNode> jsonNodeConsumer)
   {
      try (InputStream resourceStream = classForLoading.getResourceAsStream(resourcePathString))
      {
         loadInternal(resourceStream, jsonNodeConsumer);
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   public static boolean load(Path filePath, Consumer<JsonNode> jsonNodeConsumer)
   {
      try (InputStream fileStream = FileTools.newFileDataInputStream(filePath, DefaultExceptionHandler.PRINT_STACKTRACE))
      {
         loadInternal(fileStream, jsonNodeConsumer);
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }

   private static void loadInternal(InputStream fileStream, Consumer<JsonNode> jsonNodeConsumer) throws IOException
   {
      JsonFactory jsonFactory = new JsonFactory();
      ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
      JsonNode jsonNode = objectMapper.readTree(fileStream);
      jsonNodeConsumer.accept(jsonNode);
   }

   public static void load(InputStream fileStream, Consumer<JsonNode> jsonNodeConsumer)
   {
      try
      {
         JSONFileTools.loadInternal(fileStream, jsonNodeConsumer);
      }
      catch (IOException e)
      {
         LogTools.error("Could not load JSON.");
         e.printStackTrace();
      }
   }

   private static void loadInternal(URL fileURL, Consumer<JsonNode> jsonNodeConsumer) throws IOException
   {
      JsonFactory jsonFactory = new JsonFactory();
      ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
      JsonNode jsonNode = objectMapper.readTree(fileURL);
      jsonNodeConsumer.accept(jsonNode);
   }

   public static void load(URL fileURL, Consumer<JsonNode> jsonNodeConsumer)
   {
      try
      {
         JSONFileTools.loadInternal(fileURL, jsonNodeConsumer);
      }
      catch (IOException e)
      {
         LogTools.error("Could not load JSON.");
         e.printStackTrace();
      }
   }

   public static boolean saveToClasspath(String directoryNameToAssumePresent,
                                         String subsequentPathToResourceFolder,
                                         String resourcePathString,
                                         Consumer<ObjectNode> rootConsumer)
   {
      return save(WorkspacePathTools.findPathToResource(directoryNameToAssumePresent, subsequentPathToResourceFolder, resourcePathString), rootConsumer);
   }

   public static boolean save(WorkspaceFile workspaceFile, Consumer<ObjectNode> rootConsumer)
   {
      return save(workspaceFile.getFilesystemFile(), rootConsumer);
   }

   public static boolean save(Path settingsPath, Consumer<ObjectNode> rootConsumer)
   {
      if (settingsPath.getParent() != null)
      {
         FileTools.ensureDirectoryExists(settingsPath.getParent(), DefaultExceptionHandler.PRINT_STACKTRACE);
      }

      try (PrintStream printStream = new PrintStream(settingsPath.toFile()))
      {
         JsonFactory jsonFactory = new JsonFactory();
         ObjectMapper objectMapper = new ObjectMapper(jsonFactory);
         ObjectNode root = objectMapper.createObjectNode();
         rootConsumer.accept(root);
         objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, root);
         return true;
      }
      catch (IOException e)
      {
         e.printStackTrace();
         return false;
      }
   }


}
