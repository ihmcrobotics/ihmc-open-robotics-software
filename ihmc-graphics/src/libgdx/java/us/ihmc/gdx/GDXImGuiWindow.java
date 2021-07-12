package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImDrawData;
import imgui.ImGui;

public abstract class GDXImGuiWindow implements RenderableProvider
{
   private final String name;
   private Renderable renderable = null;

   public GDXImGuiWindow(String name) {
      this.name = name;
      //ImGui.createContext(); TODO bindings must exist for multiple contexts
   }

   public void update() {
      //ImGui.setContext or something TODO bindings must exist for multiple contexts
      ImGui.newFrame();

      ImGui.begin(name);
      renderImGuiWidgets(); //TODO this code will likely all need to be modified to match whatever API gets implemented
      ImGui.end();

      ImGui.render();
      render(ImGui.getDrawData());
   }

   private void render(ImDrawData data) {
      //TODO hard part
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (renderable != null)
         renderables.add(renderable);
   }

   public void dispose() {
      //ImGui.destroyContext(); TODO bindings must exist for multiple contexts
   }

   @Override
   protected void finalize() throws Throwable
   {
      this.dispose();
      super.finalize();
   }

   public abstract void renderImGuiWidgets();
}
