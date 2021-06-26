package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.HybridDirectory;
import us.ihmc.tools.io.HybridFile;
import us.ihmc.tools.io.JSONFileTools;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;
import java.util.function.Consumer;

public class ImGuiPanelManager
{
   private final ArrayList<ImGuiPanel> panels = new ArrayList<>();
   private final HybridFile panelsFile;

   public ImGuiPanelManager(HybridDirectory configurationDirectory)
   {
      panelsFile = new HybridFile(configurationDirectory, "ImGuiPanels.json");
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

   public void loadConfiguration()
   {
      JSONFileTools.loadUserWithClasspathDefaultFallback(panelsFile, jsonNode ->
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

   public void saveConfiguration(boolean saveDefault)
   {
      Consumer<ObjectNode> rootConsumer = root ->
      {
         ObjectNode anchorJSON = root.putObject("windows");

         for (ImGuiPanel panel : panels)
         {
            panel.save(anchorJSON);
         }
      };
      if (saveDefault)
      {
         JSONFileTools.save(panelsFile.getWorkspaceFile(), rootConsumer);
      }
      else
      {
         LogTools.info("Saving ImGui windows settings to {}", panelsFile.getExternalFile().toString());
         JSONFileTools.save(panelsFile.getExternalFile(), rootConsumer);
      }
   }
}
