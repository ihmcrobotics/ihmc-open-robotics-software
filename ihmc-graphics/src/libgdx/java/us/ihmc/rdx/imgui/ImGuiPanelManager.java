package us.ihmc.rdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;

import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ImGuiPanelManager
{
   private final TreeSet<ImGuiPanel> panels = new TreeSet<>(Comparator.comparing(ImGuiPanel::getPanelName));
   private final ConcurrentLinkedQueue<ImGuiPanel> removalQueue = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<ImGuiPanel> addQueue = new ConcurrentLinkedQueue<>();

   public void addPanel(ImGuiPanel panel)
   {
      panels.add(panel);
   }

   public void addPanel(String windowName, Runnable render)
   {
      panels.add(new ImGuiPanel(windowName, render));
   }

   public void removePanel(ImGuiPanel panel)
   {
      panels.remove(panel);
   }

   public void addSelfManagedPanel(String windowName)
   {
      panels.add(new ImGuiPanel(windowName));
   }

   public void queueRemovePanel(ImGuiPanel panel)
   {
      removalQueue.add(panel);
   }

   public void queueAddPanel(ImGuiPanel panel)
   {
      addQueue.add(panel);
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
      while (!removalQueue.isEmpty())
         panels.remove(removalQueue.poll());

      while (!addQueue.isEmpty())
         panels.add(addQueue.poll());

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
