package us.ihmc.rdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.behaviors.sequence.BehaviorActionDefinitionSupplier;
import us.ihmc.behaviors.sequence.BehaviorActionStateSupplier;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public abstract class RDXBehaviorAction implements BehaviorActionStateSupplier, BehaviorActionDefinitionSupplier
{
   private final MutableBoolean selected = new MutableBoolean();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper selectedWrapper = new ImBooleanWrapper(selected::booleanValue,
                                                                         selected::setValue,
                                                                         imBoolean -> ImGui.checkbox(labels.get("Selected"), imBoolean));
   private final ImBoolean expanded = new ImBoolean(true);
   private final ImString description = new ImString();
   private final ImString rejectionTooltip = new ImString();
   private int actionIndex = -1;
   private int actionNextExecutionIndex = -1;

   public void update()
   {
      update(false);
   }

   // TODO: Probably remove and use BehaviorActionDefinition#update?
   public void update(boolean concurrentActionIsNextForExecution)
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
         renderImGuiSettingWidgets();
      }
   }

   public void renderImGuiSettingWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   public abstract ImBooleanWrapper getSelected();

   public abstract ImBoolean getExpanded();

   // TODO: Put in BehaviorActionState
   public abstract String getActionTypeTitle();

   public abstract ImString getImDescription();

   public abstract ImString getRejectionTooltip();

   // TODO: Put in BehaviorActionState
   public abstract int getActionIndex();

   // TODO: Put in BehaviorActionState
   public abstract void setActionIndex(int actionIndex);

   // TODO: Put in BehaviorActionState
   public abstract int getActionNextExecutionIndex();

   // TODO: Put in BehaviorActionState
   public abstract void setActionNextExecutionIndex(int actionNextExecutionIndex);

   public ImBooleanWrapper getSelected()
   {
      return selectedWrapper;
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

   public int getActionIndex()
   {
      return actionIndex;
   }

   public void setActionIndex(int actionIndex)
   {
      this.actionIndex = actionIndex;
   }

   public int getActionNextExecutionIndex()
   {
      return actionNextExecutionIndex;
   }

   public void setActionNextExecutionIndex(int actionNextExecutionIndex)
   {
      this.actionNextExecutionIndex = actionNextExecutionIndex;
   }
}
