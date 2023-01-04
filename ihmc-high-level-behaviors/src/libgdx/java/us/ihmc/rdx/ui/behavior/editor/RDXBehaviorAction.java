package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import us.ihmc.behaviors.sequence.BehaviorActionData;
import us.ihmc.rdx.input.ImGui3DViewInput;

public abstract class RDXBehaviorAction
{
   private String nameForDisplay = "";
   private final ImBoolean selected = new ImBoolean();
   private final ImBoolean expanded = new ImBoolean(true);

   public RDXBehaviorAction()
   {

   }

   public RDXBehaviorAction(String nameForDisplay)
   {
      this.nameForDisplay = nameForDisplay;
   }

   public void update()
   {

   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   public final void renderImGuiWidgets()
   {
      if (expanded.get())
      {
         renderImGuiSettingWidgets();
      }
   }

   public void renderImGuiSettingWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public abstract BehaviorActionData getActionData();

   public ImBoolean getSelected()
   {
      return selected;
   }

   public ImBoolean getExpanded()
   {
      return expanded;
   }

   public String getNameForDisplay()
   {
      return nameForDisplay;
   }
}
