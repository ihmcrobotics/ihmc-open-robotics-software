package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.sequence.BehaviorActionDefinitionSupplier;
import us.ihmc.behaviors.sequence.BehaviorActionStateSupplier;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXBehaviorAction implements BehaviorActionStateSupplier, BehaviorActionDefinitionSupplier
{
   private transient final RDXBehaviorActionSequenceEditor editor;
   private final ImBoolean selected = new ImBoolean();
   private final ImBoolean expanded = new ImBoolean(true);
   private final ImString description = new ImString();
   private final ImString rejectionTooltip = new ImString();

   public RDXBehaviorAction(RDXBehaviorActionSequenceEditor editor)
   {
      this.editor = editor;
   }

   public void update()
   {

   }

   /** @deprecated TODO: Figure out how to remove this. */
   public void updateBeforeRemoving()
   {

   }

   public void calculateVRPick(RDXVRContext vrContext)
   {

   }

   public void processVRInput(RDXVRContext vrContext)
   {

   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   public void renderImGuiWidgets()
   {
      if (expanded.get())
      {
         renderImGuiWidgetsInternal();
      }
   }

   protected void renderImGuiWidgetsInternal()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public abstract String getActionTypeTitle();

   public ImBoolean getSelected()
   {
      return selected;
   }

   public ImBoolean getExpanded()
   {
      return expanded;
   }

   public ImString getDescription()
   {
      return description;
   }

   public ImString getRejectionTooltip()
   {
      return rejectionTooltip;
   }

   public RDXBehaviorActionSequenceEditor getEditor()
   {
      return editor;
   }
}
