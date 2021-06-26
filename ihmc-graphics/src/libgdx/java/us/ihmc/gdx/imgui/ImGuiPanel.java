package us.ihmc.gdx.imgui;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;

import java.util.ArrayList;
import java.util.Map;

public class ImGuiPanel extends ImGuiPanelSizeHandler
{
   private final String panelName;
   private Runnable render;
   private final ImBoolean isShowing;
   private final ArrayList<ImGuiPanel> children = new ArrayList<>();

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
      this.panelName = panelName;
      this.render = render;
      this.isShowing = new ImBoolean(isShowing);
   }

   /* package-private */ void renderMenuItem()
   {
      if (isTogglable())
      {
         ImGui.menuItem(panelName, "", isShowing);

         for (ImGuiPanel child : children)
         {
            child.renderMenuItem();
         }
      }
   }

   /* package-private */ void renderPanelAndChildren()
   {
      if (isTogglable() && isShowing.get())
      {
         handleSizeBeforeBegin();
         ImGui.begin(panelName, isShowing);
         handleSizeAfterBegin();
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
         isShowing.set(panelEntry.getValue().asBoolean());
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
         anchorJSON.put(panelName, isShowing.get());
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

   public ImBoolean getIsShowing()
   {
      return isShowing;
   }

   public String getPanelName()
   {
      return panelName;
   }
}
