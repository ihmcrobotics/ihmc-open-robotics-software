package us.ihmc.rdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.flag.ImGuiWindowFlags;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.log.LogTools;

import java.util.Comparator;
import java.util.Map;
import java.util.TreeSet;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ImGuiPanel extends ImGuiPanelSizeHandler
{
   private final String panelName;
   private Runnable render;
   private final ImBoolean isShowing;
   private final boolean hasMenuBar;
   private final TreeSet<ImGuiPanel> children = new TreeSet<>(Comparator.comparing(ImGuiPanel::getPanelName));
   private final ConcurrentLinkedQueue<ImGuiPanel> removalQueue = new ConcurrentLinkedQueue<>();
   private final ConcurrentLinkedQueue<ImGuiPanel> additionQueue = new ConcurrentLinkedQueue<>();

   private int lastDockID = -1;

   public ImGuiPanel(String panelName)
   {
      this(panelName, null, false);
   }

   public ImGuiPanel(String panelName, Runnable render)
   {
      this(panelName, render, false);
   }

   public ImGuiPanel(String panelName, Runnable render, boolean isShowing)
   {
      this(panelName, render, isShowing, false);
   }

   public ImGuiPanel(String panelName, Runnable render, boolean isShowing, boolean hasMenuBar)
   {
      this.panelName = panelName;
      this.render = render;
      this.isShowing = new ImBoolean(isShowing);
      this.hasMenuBar = hasMenuBar;
   }

   /* package-private */ void renderMenuItem()
   {
      renderMenuItem("");
   }
   /* package-private */ void renderMenuItem(String indent)
   {
      ImGui.menuItem(indent + panelName, "", isShowing);

      for (ImGuiPanel child : children)
      {
         child.renderMenuItem(indent + "\t");
      }
   }

   /* package-private */ void renderPanelAndChildren(ImGuiDockspacePanel justClosedPanel)
   {
      while (!removalQueue.isEmpty())
         children.remove(removalQueue.poll());
      while (!additionQueue.isEmpty())
         children.add(additionQueue.poll());

      if (isShowing.get() && render != null)
      {
         handleSizeBeforeBegin();
         int windowFlags = ImGuiWindowFlags.None;
         if (hasMenuBar)
            windowFlags |= ImGuiWindowFlags.MenuBar;
         ImGui.begin(panelName, isShowing, windowFlags);
         handleSizeAfterBegin();

         int windowDockID = ImGui.getWindowDockID();
         if (lastDockID != windowDockID)
         {
            LogTools.debug("Dock ID changed. {}: {} -> {}", panelName, lastDockID, windowDockID);
            if (justClosedPanel != null)
            {
               LogTools.info("Closing \"{}\" because containing dockspace \"{}\" closed", panelName, justClosedPanel.getName());
               isShowing.set(false);
            }
         }
         lastDockID = windowDockID;

         render.run();
         ImGui.end();
      }

      for (ImGuiPanel child : children)
      {
         child.renderPanelAndChildren(justClosedPanel);
      }
   }

   public void addChild(ImGuiPanel child)
   {
      children.add(child);
   }

   public void queueRemoveChild(ImGuiPanel panel)
   {
      removalQueue.add(panel);

   }
   public void queueAddChild(ImGuiPanel panel)
   {
      additionQueue.add(panel);
   }

   /* package-private */ void load(Map.Entry<String, JsonNode> panelEntry)
   {
      if (panelName.equals(panelEntry.getKey()))
      {
         isShowing.set(panelEntry.getValue().asBoolean());
      }

      for (ImGuiPanel child : children)
      {
         child.load(panelEntry);
      }
   }

   /* package-private */ void save(ObjectNode anchorJSON)
   {
      anchorJSON.put(panelName, isShowing.get());

      for (ImGuiPanel child : children)
      {
         child.save(anchorJSON);
      }
   }

   public void setRenderMethod(Runnable render)
   {
      this.render = render;
   }

   public ImBoolean getIsShowing()
   {
      return isShowing;
   }

   public String getPanelName()
   {
      return panelName;
   }

   public TreeSet<ImGuiPanel> getChildren()
   {
      return children;
   }
}
