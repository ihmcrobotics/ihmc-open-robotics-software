package us.ihmc.gdx.imgui;

import imgui.type.ImBoolean;

public class ImGuiPanel
{
   private final String panelName;
   private final Runnable render;
   private final ImBoolean enabled;

   public ImGuiPanel(String panelName, Runnable render, boolean enabled)
   {
      this.panelName = panelName;
      this.render = render;
      this.enabled = new ImBoolean(enabled);
   }

   public ImGuiPanel(String panelName, Runnable render)
   {
      this.panelName = panelName;
      this.render = render;
      this.enabled = new ImBoolean(false);
   }

   public ImGuiPanel(String panelName)
   {
      this.panelName = panelName;
      this.render = null;
      this.enabled = null;
   }

   public void render()
   {
      if (enabled.get())
      {
         render.run();
      }
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
}
