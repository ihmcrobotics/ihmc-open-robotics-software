package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;
import java.util.function.Consumer;

public class ImGuiPanelManager
{
   private final ArrayList<ImGuiWindow> windows = new ArrayList<>();

   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;

   public ImGuiPanelManager(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
   }

   public void addPanel(String windowName, Runnable render)
   {
      windows.add(new ImGuiWindow(windowName, render));
   }

   public void addPrimaryPanel(String windowName)
   {
      windows.add(new ImGuiWindow(windowName));
   }

   public void loadConfiguration(Path settingsPath)
   {
      Path windowsSettingsPath = settingsPath.getParent().resolve(settingsPath.getFileName().toString().replace("Settings.ini", "Panels.json"));
      JSONFileTools.loadWithClasspathDefault(windowsSettingsPath,
                                             classForLoading,
                                             directoryNameToAssumePresent,
                                             subsequentPathToResourceFolder,
                                             "/imgui",
                                             jsonNode ->
      {
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
      });
   }

   public void saveConfiguration(Path settingsPath, boolean saveDefault)
   {
      Consumer<ObjectNode> rootConsumer = root ->
      {
         ObjectNode anchorJSON = root.putObject("windows");

         for (ImGuiWindow window : this.windows)
         {
            if (window.isTogglable())
            {
               anchorJSON.put(window.getWindowName(), window.getEnabled().get());
            }
         }
      };
      String saveFileNameString = settingsPath.getFileName().toString().replace("Settings.ini", "Panels.json");
      if (saveDefault)
      {
         JSONFileTools.saveToClasspath(directoryNameToAssumePresent, subsequentPathToResourceFolder, "imgui/" + saveFileNameString, rootConsumer);
      }
      else
      {
         Path windowsSettingsPath = settingsPath.getParent().resolve(saveFileNameString);
         LogTools.info("Saving ImGui windows settings to {}", windowsSettingsPath.toString());
         JSONFileTools.save(windowsSettingsPath, rootConsumer);
      }
   }

   public ArrayList<ImGuiWindow> getWindows()
   {
      return windows;
   }
}
