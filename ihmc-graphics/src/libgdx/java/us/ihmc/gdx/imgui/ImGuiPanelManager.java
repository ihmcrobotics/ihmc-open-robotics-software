package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.JSONFileTools;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.Map;
import java.util.function.Consumer;

public class ImGuiPanelManager
{
   private final ArrayList<ImGuiPanel> panels = new ArrayList<>();

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

   public void renderPanels(ImGuiDockspacePanel justClosedPanel)
   {
      for (ImGuiPanel panel : panels)
      {
         panel.renderPanelAndChildren(justClosedPanel);
      }
   }

   public void loadConfiguration(JsonNode jsonNode)
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
   }

   public void saveConfiguration(ObjectNode root)
   {
      ObjectNode anchorJSON = root.putObject("windows");

      for (ImGuiPanel panel : panels)
      {
         panel.save(anchorJSON);
      }
   }
}
