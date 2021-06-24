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
   private final ArrayList<ImGuiPanel> panels = new ArrayList<>();

   private final Class<?> classForLoading;
   private final String directoryNameToAssumePresent;
   private final String subsequentPathToResourceFolder;

   public ImGuiPanelManager(Class<?> classForLoading, String directoryNameToAssumePresent, String subsequentPathToResourceFolder)
   {
      this.classForLoading = classForLoading;
      this.directoryNameToAssumePresent = directoryNameToAssumePresent;
      this.subsequentPathToResourceFolder = subsequentPathToResourceFolder;
   }

   public void addPanel(ImGuiPanel panel)
   {
      panels.add(panel);
   }

   public void addPanel(String windowName, Runnable render)
   {
      panels.add(new ImGuiPanel(windowName, render));
   }

   public void addPrimaryPanel(String windowName)
   {
      panels.add(new ImGuiPanel(windowName));
   }

   public void renderPanelMenu()
   {
      for (ImGuiPanel panel : panels)
      {
         panel.renderMenuItem();
      }
   }

   public void renderPanels()
   {
      for (ImGuiPanel panel : panels)
      {
         panel.renderPanelAndChildren();
      }
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
            Map.Entry<String, JsonNode> panelEntry = it.next();
            for (ImGuiPanel panel : panels)
            {
               panel.load(panelEntry);
            }
         }
      });
   }

   public void saveConfiguration(Path settingsPath, boolean saveDefault)
   {
      Consumer<ObjectNode> rootConsumer = root ->
      {
         ObjectNode anchorJSON = root.putObject("windows");

         for (ImGuiPanel panel : panels)
         {
            panel.save(anchorJSON);
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

   public ArrayList<ImGuiPanel> getPanels()
   {
      return panels;
   }
}
