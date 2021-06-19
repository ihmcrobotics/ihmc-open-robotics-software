package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.flag.ImGuiCond;
import imgui.type.ImBoolean;

import java.util.ArrayList;
import java.util.Map;

public class ImGuiPanel
{
   private final String panelName;
   private int firstTimeWidth = 300;
   private int firstTimeHeight = 200;
   private Runnable render;
   private final ImBoolean enabled;

   private final ArrayList<ImGuiPanel> children = new ArrayList<>();

   public ImGuiPanel(String panelName)
   {
      this(panelName, null, false);
   }

   public ImGuiPanel(String panelName, Runnable render)
   {
      this(panelName, render, false);
   }

   public ImGuiPanel(String panelName, Runnable render, boolean enabled)
   {
      this.panelName = panelName;
      this.render = render;
      this.enabled = new ImBoolean(enabled);
   }

   /* package-private */ void renderMenuItem()
   {
      if (isTogglable())
      {
         ImGui.menuItem(panelName, "", enabled);

         for (ImGuiPanel child : children)
         {
            child.renderMenuItem();
         }
      }
   }

   /* package-private */ void renderPanelAndChildren()
   {
      if (isTogglable() && enabled.get())
      {
         ImGui.setNextWindowSize(firstTimeWidth, firstTimeHeight, ImGuiCond.FirstUseEver);
         ImGui.begin(panelName, enabled);
         render.run();
         ImGui.end();
      }

      for (ImGuiPanel child : children)
      {
         child.renderPanelAndChildren();
      }
   }

   public void addChild(ImGuiPanel child)
   {
      children.add(child);
   }

   /* package-private */ void load(Map.Entry<String, JsonNode> panelEntry)
   {
      if (panelName.equals(panelEntry.getKey()))
      {
         enabled.set(panelEntry.getValue().asBoolean());
      }

      for (ImGuiPanel child : children)
      {
         child.load(panelEntry);
      }
   }

   /* package-private */ void save(ObjectNode anchorJSON)
   {
      if (isTogglable())
      {
         anchorJSON.put(panelName, enabled.get());
      }

      for (ImGuiPanel child : children)
      {
         child.save(anchorJSON);
      }
   }

   public void setRenderMethod(Runnable render)
   {
      this.render = render;
   }

   public boolean isTogglable()
   {
      return render != null;
   }

   public ImBoolean getEnabled()
   {
      return enabled;
   }

   public String getPanelName()
   {
      return panelName;
   }

   public void setFirstTimeWidth(int firstTimeWidth)
   {
      this.firstTimeWidth = firstTimeWidth;
   }

   public void setFirstTimeHeight(int firstTimeHeight)
   {
      this.firstTimeHeight = firstTimeHeight;
   }
}
