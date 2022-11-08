package us.ihmc.rdx.ui.visualizers;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public abstract class RDXVisualizer implements RenderableProvider
{
   private ImBoolean active = new ImBoolean(false);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String title;
   private boolean createdYet = false;

   public RDXVisualizer(String title)
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
      ImGui.checkbox(labels.get(title), active);
      ImGuiTools.previousWidgetTooltip("Active");
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

   public String getTitle()
   {
      return title;
   }
}
