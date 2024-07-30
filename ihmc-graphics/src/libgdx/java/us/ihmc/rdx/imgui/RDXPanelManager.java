package us.ihmc.rdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import gnu.trove.map.hash.TIntObjectHashMap;

import java.util.*;
import java.util.concurrent.ConcurrentLinkedQueue;

public class RDXPanelManager
{
   private final TreeSet<RDXPanel> panels = new TreeSet<>(Comparator.comparing(RDXPanel::getPanelName));
   private final ConcurrentLinkedQueue<RDXPanel> removalQueue = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<RDXPanel> addQueue = new ConcurrentLinkedQueue<>();

   public Collection<RDXPanel> getPanels()
   {
      return panels;
   }

   public void addPanel(RDXPanel panel)
   {
      panels.add(panel);
   }

   public void addPanel(String windowName, Runnable render)
   {
      panels.add(new RDXPanel(windowName, render));
   }

   public void addSelfManagedPanel(String windowName)
   {
      panels.add(new RDXPanel(windowName));
   }

   public void queueRemovePanel(RDXPanel panel)
   {
      removalQueue.add(panel);
   }

   public void queueAddPanel(RDXPanel panel)
   {
      addQueue.add(panel);
   }

   public void renderPanelMenu()
   {
      for (RDXPanel panel : panels)
      {
         panel.renderMenuItem();
      }
   }

   public void renderPanels(TIntObjectHashMap<RDXDockspacePanel> dockIDMap, boolean lockPanelsWithinViewports)
   {
      while (!removalQueue.isEmpty())
         panels.remove(removalQueue.poll());

      while (!addQueue.isEmpty())
         panels.add(addQueue.poll());

      for (RDXPanel panel : panels)
      {
         panel.renderPanelAndChildren(dockIDMap, lockPanelsWithinViewports);
      }
   }

   public void loadConfiguration(JsonNode jsonNode)
   {
      JsonNode windowsNode = jsonNode.get("windows");
      for (Iterator<Map.Entry<String, JsonNode>> it = windowsNode.fields(); it.hasNext(); )
      {
         Map.Entry<String, JsonNode> panelEntry = it.next();
         for (RDXPanel panel : panels)
         {
            panel.load(panelEntry);
         }
      }
   }

   public void saveConfiguration(ObjectNode root)
   {
      ObjectNode anchorJSON = root.putObject("windows");

      for (RDXPanel panel : panels)
      {
         panel.save(anchorJSON);
      }
   }
}
