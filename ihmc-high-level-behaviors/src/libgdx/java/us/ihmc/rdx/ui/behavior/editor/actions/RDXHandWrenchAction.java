package us.ihmc.rdx.ui.behavior.editor.actions;

import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImString;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionDefinition;
import us.ihmc.behaviors.sequence.actions.HandWrenchActionState;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.behavior.editor.RDXBehaviorAction;

public class RDXHandWrenchAction extends RDXBehaviorAction
{
   private final HandWrenchActionState state = new HandWrenchActionState();
   private final HandWrenchActionDefinition definition = state.getDefinition();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImDoubleWrapper trajectoryDurationWidget = new ImDoubleWrapper(definition::getTrajectoryDuration,
                                                                                definition::setTrajectoryDuration,
                                                                                imDouble -> ImGui.inputDouble(labels.get("Trajectory duration"), imDouble));
   private final ImDoubleWrapper forceWidget = new ImDoubleWrapper(definition::getForce,
                                                                   definition::setForce,
                                                                   imDouble -> ImGui.inputDouble(labels.get("Force"), imDouble));

   @Override
   public void renderImGuiWidgets()
   {
      rdxActionBasics.renderImGuiWidgets();
   }

   @Override
   public void renderImGuiSettingWidgets()
   {
      ImGui.pushItemWidth(80.0f);
      trajectoryDurationWidget.renderImGuiWidget();
      forceWidget.renderImGuiWidget();
      ImGui.popItemWidth();
   }

   @Override
   public String getActionTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Hand Wrench";
   }

   @Override
   public ImBooleanWrapper getSelected()
   {
      return rdxActionBasics.getSelected();
   }

   @Override
   public ImBoolean getExpanded()
   {
      return rdxActionBasics.getExpanded();
   }

   @Override
   public ImString getImDescription()
   {
      return rdxActionBasics.getDescription();
   }

   @Override
   public ImString getRejectionTooltip()
   {
      return rdxActionBasics.getRejectionTooltip();
   }

   @Override
   public int getActionIndex()
   {
      return rdxActionBasics.getActionIndex();
   }

   @Override
   public void setActionIndex(int actionIndex)
   {
      rdxActionBasics.setActionIndex(actionIndex);
   }

   @Override
   public int getActionNextExecutionIndex()
   {
      return rdxActionBasics.getActionNextExecutionIndex();
   }

   @Override
   public void setActionNextExecutionIndex(int actionNextExecutionIndex)
   {
      rdxActionBasics.setActionNextExecutionIndex(actionNextExecutionIndex);
   }

   @Override
   public HandWrenchActionState getState()
   {
      return state;
   }

   @Override
   public HandWrenchActionDefinition getDefinition()
   {
      return definition;
   }
}
