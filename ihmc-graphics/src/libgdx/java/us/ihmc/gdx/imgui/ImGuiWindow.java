package us.ihmc.gdx.imgui;

import imgui.type.ImBoolean;

public class ImGuiWindow
{
   private final String windowName;
   private final Runnable render;
   private final ImBoolean enabled;

   public ImGuiWindow(String windowName, Runnable render, boolean enabled)
   {
      this.windowName = windowName;
      this.render = render;
      this.enabled = new ImBoolean(enabled);
   }

   public ImGuiWindow(String windowName, Runnable render)
   {
      this.windowName = windowName;
      this.render = render;
      this.enabled = new ImBoolean(false);
   }

   public ImGuiWindow(String windowName)
   {
      this.windowName = windowName;
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

   public String getWindowName()
   {
      return windowName;
   }
}
