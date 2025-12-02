package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionState;
import us.ihmc.handsros2.abilityHand.AbilityHandManager;
import us.ihmc.handsros2.abilityHand.AbilityHandManager.ControlMode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.widgets.ImGuiHandWidget;
import us.ihmc.robotics.EuclidCoreMissingTools;

public class RDXAbilityHandAction extends RDXActionNode<AbilityHandActionState, AbilityHandActionDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImGuiHandWidget handWidget = new ImGuiHandWidget();

   private static final String[] FINGER_NAMES = {"Index", "Middle", "Ring", "Pinky", "Flex", "Rotator"};

   private final ImFloat[] sliderPositions = new ImFloat[6];
   private final ImFloat[] sliderVelocities = new ImFloat[6];

   public RDXAbilityHandAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);

      for (int i = 0; i < 6; i++)
      {
         sliderPositions[i] = new ImFloat();
         sliderVelocities[i] = new ImFloat();
      }
   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.text("Ability Hand action for " + definition.getSide().getLowerCaseName() + " side");

      ImGui.text("Control Mode:");
      if (ImGui.radioButton(labels.get(ControlMode.GRIP.name()), definition.getControlMode() == ControlMode.GRIP))
      {
         definition.setControlMode(ControlMode.GRIP);
      }
      ImGui.sameLine();
      if (ImGui.radioButton(labels.get(ControlMode.POSITION.name()), definition.getControlMode() == ControlMode.POSITION))
      {
         definition.setControlMode(ControlMode.POSITION);
      }

      if (definition.getControlMode() == ControlMode.GRIP)
      {
         if (ImGui.beginCombo(labels.get("Grip"), definition.getGrip().name()))
         {
            for (AbilityHandManager.Grip grip : AbilityHandManager.Grip.values)
            {
               boolean isSelected = definition.getGrip() == grip;
               if (ImGui.selectable(grip.name(), isSelected))
               {
                  definition.setGrip(grip);
               }
               if (isSelected)
               {
                  ImGui.setItemDefaultFocus();
               }
            }
            ImGui.endCombo();
         }
      }
      else
      {
         for (int i = 0; i < 6; i++)
         {
            sliderPositions[i].set(definition.getGoalPositions().getValueReadOnly(i));
            sliderVelocities[i].set(definition.getGoalVelocities().getValueReadOnly(i));

            float sliderMax = i == 5 ? -120.0f : 120.0f; // thumb rotator moves negative
            ImGui.pushItemWidth(ImGui.getColumnWidth() * 0.6f);
            if (ImGui.sliderFloat(labels.getHidden(FINGER_NAMES[i]), sliderPositions[i].getData(), 0.0f, sliderMax,
                                  "%s: %.2f%s flexion".formatted(FINGER_NAMES[i], sliderPositions[i].get(), EuclidCoreMissingTools.DEGREE_SYMBOL)))
               definition.getGoalPositions().setValue(i, sliderPositions[i].get());
            ImGui.popItemWidth();
            ImGui.sameLine();
            ImGui.pushItemWidth(ImGui.getColumnWidth());
            if (ImGui.inputFloat(labels.getHidden("Velocity" + i), sliderVelocities[i], 0.1f, 1.0f, "%.2f deg/s"))
               definition.getGoalVelocities().setValue(i, sliderVelocities[i].get());
            ImGui.popItemWidth();
         }
      }
   }

   @Override
   public void renderTreeViewRow()
   {
      super.renderRowBeginning();
      super.renderEditableName();

      ImGui.sameLine();
      handWidget.render(definition.getSide(), ImGui.getFrameHeight(), false);
   }

   @Override
   public String getLeafTypeTitle()
   {
      return definition.getSide().getPascalCaseName() + " Ability Hand";
   }
}
