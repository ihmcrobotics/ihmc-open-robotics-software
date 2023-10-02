package us.ihmc.rdx.ui.behavior.editor;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import org.apache.commons.lang3.mutable.MutableBoolean;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

/**
 * The UI representation of a robot behavior action. It provides a base
 * template for implementing an interactable action.
 */
public class RDXBehaviorActionBasics
{
   private final RDXBehaviorAction rdxBehaviorAction;
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

   public RDXBehaviorActionBasics(RDXBehaviorAction rdxBehaviorAction)
   {
      this.rdxBehaviorAction = rdxBehaviorAction;
   }

   public final void renderImGuiWidgets()
   {
      if (expanded.get())
      {
         rdxBehaviorAction.renderImGuiSettingWidgets();
      }
   }

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
