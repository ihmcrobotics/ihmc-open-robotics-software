package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;

import java.util.Set;

public abstract class RDXVisualizer implements RDXRenderableProvider
{
   private final ImBoolean active = new ImBoolean(false);
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final String title;
   private boolean createdYet = false;
   private Set<RDXSceneLevel> sceneLevels = Set.of(RDXSceneLevel.MODEL);

   public RDXVisualizer(String title)
   {
      this.title = ImGuiTools.uniqueLabel(title);
   }

   public void create()
   {
      createdYet = true;
      if (getPanel() != null)
      {
         setActive(getPanel().getIsShowing().get());
      }
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.checkbox(labels.get(title), active))
      {
         setActive(active.get());
      }
      ImGuiTools.previousWidgetTooltip("Active");
   }

   /**
    * Only called when active.
    */
   public void update()
   {
      if (!createdYet)
      {
         create();
      }
   }

   /**
    * It is assumed by extending classes that this will be called when the active
    * state changes.
    */
   public void setActive(boolean active)
   {
      this.active.set(active);
   }

   public boolean isActive()
   {
      return active.get();
   }

   /**
    * Only called when active.
    */
   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {

   }

   public void setSceneLevels(RDXSceneLevel... sceneLevels)
   {
      this.sceneLevels = Set.of(sceneLevels);
   }

   public boolean sceneLevelCheck(Set<RDXSceneLevel> sceneLevels)
   {
      for (RDXSceneLevel sceneLevel : this.sceneLevels)
         if (sceneLevels.contains(sceneLevel))
            return true;
      return false;
   }

   public RDXPanel getPanel()
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
