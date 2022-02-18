package us.ihmc.gdx;

public class GDX3DSituatedImGuiPanel
{
   private final String name;
   private final Runnable render;

   public GDX3DSituatedImGuiPanel(String name, Runnable render)
   {
      this.name = name;
      this.render = render;
   }

   public void renderImGuiWidgets()
   {
      render.run();
   }

   public final String getPanelName()
   {
      return name;
   }
}
