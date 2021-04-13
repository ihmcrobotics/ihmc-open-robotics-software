package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImInt;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.log.LogTools;

import java.io.BufferedInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

public class ImGuiDockingSetup
{
   private final ArrayList<ImGuiDockingSetupInstruction> instructions = new ArrayList<>();
   private final HashMap<String, Integer> windowsSpaceIds = new HashMap<>();
   private final ArrayList<ImGuiWindow> windows = new ArrayList<>();
   private Class<?> classForLoading;
   private String directoryNameToAssumePresent;
   private String subsequentPathToResourceFolder;
   private boolean loadSaveEnabled = false;

   public ImGuiDockingSetup()
   {
   }

   public ImGuiDockingSetup(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
      loadSaveEnabled = true;
   }

   public void addFirst(String windowName)
   {
      addFirst(new ImGuiWindow(windowName));
   }

   public void addFirst(ImGuiWindow window)
   {
      addInstruction(new ImGuiDockingSetupInstruction(window.getWindowName()), window);
   }

   public void splitAdd(String windowToAddName, int imGuiDir, double percent)
   {
      splitAdd(new ImGuiWindow(windowToAddName), imGuiDir, percent);
   }

   public void splitAdd(ImGuiWindow windowToAdd, int imGuiDir, double percent)
   {
      splitAdd(windows.get(windows.size() - 1).getWindowName(), windowToAdd, imGuiDir, percent);
   }

   public void splitAdd(String windowToSplitName, String windowToAddName, int imGuiDir, double percent)
   {
      splitAdd(windowToSplitName, new ImGuiWindow(windowToAddName), imGuiDir, percent);
   }

   public void splitAdd(String windowToSplitName, ImGuiWindow windowToAdd, int imGuiDir, double percent)
   {
      addInstruction(new ImGuiDockingSetupInstruction(windowToSplitName, windowToAdd.getWindowName(), imGuiDir, percent), windowToAdd);
   }

   private void addInstruction(ImGuiDockingSetupInstruction instruction, ImGuiWindow window)
   {
      instructions.add(instruction);
      windows.add(window);
   }

   public void build(int centralDockspaceId)
   {
      for (int i = 0; i < instructions.size(); i++)
      {
         ImGuiDockingSetupInstruction instruction = instructions.get(i);
         if (i == 0)
         {
            String windowName = instruction.getWindowName();
            ImGui.dockBuilderDockWindow(windowName, centralDockspaceId);
            windowsSpaceIds.put(windowName, centralDockspaceId);
         }
         else
         {
            String windowToSplit = instruction.getWindowToSplit();
            int imGuiDir = instruction.getImGuiDir();
            double percent = instruction.getPercent();
            String windowNameToAdd = instruction.getWindowNameToAdd();
            int dockspaceToSplit = windowsSpaceIds.get(windowToSplit);
            ImInt newIdForExistingWindow = new ImInt();
            int dockspaceIdForNewWindow = ImGui.dockBuilderSplitNode(dockspaceToSplit, imGuiDir, (float) percent, null, newIdForExistingWindow);
            windowsSpaceIds.put(windowToSplit, newIdForExistingWindow.get());
            windowsSpaceIds.put(windowNameToAdd, dockspaceIdForNewWindow);
         }
      }

      for (String windowName : windowsSpaceIds.keySet())
      {
         int dockspaceId = windowsSpaceIds.get(windowName);
         ImGui.dockBuilderDockWindow(windowName, dockspaceId);
      }

      ImGui.dockBuilderFinish(centralDockspaceId);
   }

   public void loadConfiguration(Path settingsPath)
   {
      if (loadSaveEnabled)
      {
         Path windowsSettingsPath = settingsPath.getParent().resolve(settingsPath.getFileName().toString().replace("Settings.ini", "Windows.ini"));
         InputStream settingsStream;
         if (!Files.exists(windowsSettingsPath))
         {
            String resourcePathString = "imgui/" + windowsSettingsPath.getFileName().toString();
            settingsStream = classForLoading.getResourceAsStream(resourcePathString);
            if (settingsStream == null)
            {
               Path resourceFolder = PathTools.findDirectoryInline(directoryNameToAssumePresent)
                                              .resolve(subsequentPathToResourceFolder)
                                              .resolve(resourcePathString)
                                              .toAbsolutePath()
                                              .normalize();
               LogTools.warn("No settings found. Please save defaults to {}", resourceFolder.toString());
               return;
            }
            else
            {
               LogTools.info("No settings found. Loading defaults from {}", resourcePathString);
            }
         }
         else
         {
            LogTools.info("Loading ImGui windows settings from {}", windowsSettingsPath.toString());
            settingsStream = FileTools.newFileDataInputStream(windowsSettingsPath, DefaultExceptionHandler.PRINT_STACKTRACE);
         }

         JsonFactory jsonFactory = new JsonFactory();
         ObjectMapper objectMapper = new ObjectMapper(jsonFactory);

         try
         {
            JsonNode jsonNode = objectMapper.readTree(settingsStream);
            JsonNode windowsNode = jsonNode.get("windows");
            for (Iterator<Map.Entry<String, JsonNode>> it = windowsNode.fields(); it.hasNext(); )
            {
               Map.Entry<String, JsonNode> window = it.next();
               for (ImGuiWindow imGuiWindow : windows)
               {
                  if (imGuiWindow.getWindowName().equals(window.getKey()))
                  {
                     imGuiWindow.getEnabled().set(window.getValue().asBoolean());
                  }
               }
            }
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   public void saveConfiguration(Path settingsPath)
   {
      if (loadSaveEnabled)
      {
         Path windowsSettingsPath = settingsPath.getParent().resolve(settingsPath.getFileName().toString().replace("Settings.ini", "Windows.ini"));
         String windowsSettingsPathString = windowsSettingsPath.toString();
         LogTools.info("Saving ImGui windows settings to {}", windowsSettingsPathString);
         PrintStream printStream = null;
         try
         {
            printStream = new PrintStream(windowsSettingsPath.toFile());
            JsonFactory jsonFactory = new JsonFactory();
            ObjectMapper objectMapper = new ObjectMapper(jsonFactory);

            ObjectNode root = objectMapper.createObjectNode();
            ObjectNode anchorJSON = root.putObject("windows");

            for (ImGuiWindow window : windows)
            {
               if (window.isTogglable())
               {
                  anchorJSON.put(window.getWindowName(), window.getEnabled().get());
               }
            }

            objectMapper.writerWithDefaultPrettyPrinter().writeValue(printStream, root);
            printStream.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
            if (printStream != null)
               printStream.close();
         }
      }
   }

   public ArrayList<ImGuiWindow> getWindows()
   {
      return windows;
   }
}
