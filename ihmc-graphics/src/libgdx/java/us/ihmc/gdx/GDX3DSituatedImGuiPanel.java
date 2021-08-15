package us.ihmc.gdx;

public abstract class GDX3DSituatedImGuiPanel
{
   private final String name;

   public GDX3DSituatedImGuiPanel(String name)
   {
      this.name = name;
      GDX3DSituatedImGuiPanelManager.getInstance().addPanel(this);
   }

   public abstract void renderImGuiWidgets();

   public void dispose()
   {

   }

   public final String getName()
   {
      return name;
   }

   @Override
   protected final void finalize() throws Throwable
   {
      this.dispose();
      super.finalize();
   }
}
