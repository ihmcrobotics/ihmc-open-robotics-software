package us.ihmc.gdx.ui.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiTools;

public abstract class ImGuiGDXVisualizer implements RenderableProvider
{
   private ImBoolean active = new ImBoolean(false);
   private final String title;
   private boolean activeChanged = false;
   private boolean createdYet = false;

   public ImGuiGDXVisualizer(String title)
   {
      this.title = ImGuiTools.uniqueLabel(title);
   }

   public void create()
   {
      createdYet = true;
      if (getPanel() != null)
      {
         active = getPanel().getIsShowing();
      }
   }

   public void renderImGuiWidgets()
   {
      activeChanged = ImGui.checkbox(title, active);
   }

   public void update()
   {
      if (!createdYet)
      {
         create();
      }
   }

   public void setActive(boolean active)
   {
      this.active.set(active);
   }

   public boolean isActive()
   {
      return active.get();
   }

   public boolean getActiveChanged()
   {
      return activeChanged;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public ImGuiPanel getPanel()
   {
      return null;
   }

   public void destroy()
   {

   }
}
