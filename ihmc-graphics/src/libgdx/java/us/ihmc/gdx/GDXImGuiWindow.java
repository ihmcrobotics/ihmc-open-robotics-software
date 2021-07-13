package us.ihmc.gdx;

public abstract class GDXImGuiWindow
{
   private final String name;

   public GDXImGuiWindow(String name) {
      this.name = name;
      GDXImGuiWindowManager.getInstance().addPanel(this);
   }

   public abstract void renderImGuiWidgets();

   public void dispose() {}

   public final String getName() {
      return name;
   }

   @Override
   protected final void finalize() throws Throwable
   {
      this.dispose();
      super.finalize();
   }
}
