package us.ihmc.rdx.behaviorTree.actions;

import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionDefinition;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionDefinition.SuccessCriteria;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionState;
import us.ihmc.handsros2.abilityHand.AbilityHandGrip;
import us.ihmc.handsros2.abilityHand.AbilityHandControlMode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImFloatWrapper;
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
   private final AbilityHandControlMode[] modes = {AbilityHandControlMode.GRIP, AbilityHandControlMode.POSITION };
   private final ImFloatWrapper ultimateTimeoutWidget;
   private final ImBooleanWrapper enableWiggleOnFailureWidget;
   private final ImFloatWrapper timeToWiggleWidget;
   private final ImFloatWrapper eachJointPositionToleranceWidget;
   private final ImFloatWrapper sufficientCumulativeJointMovementWidget;

   public RDXAbilityHandAction(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new AbilityHandActionState(id, rootNode.getState()), rootNode);

      for (int i = 0; i < 6; i++)
      {
         sliderPositions[i] = new ImFloat();
         sliderVelocities[i] = new ImFloat();
      }

      ultimateTimeoutWidget = new ImFloatWrapper(definition::getUltimateTimeout,
                                                 definition::setUltimateTimeout,
                                                 imFloat -> ImGui.inputFloat(labels.get("Ultimate timeout"), imFloat));
      enableWiggleOnFailureWidget = new ImBooleanWrapper(definition::getEnableWiggleOnFailure,
                                                         definition::setEnableWiggleOnFailure,
                                                         imBoolean -> ImGui.checkbox(labels.get("Enable wiggle on failure"), imBoolean));
      timeToWiggleWidget = new ImFloatWrapper(definition::getTimeToWiggle,
                                              definition::setTimeToWiggle,
                                              imFloat -> ImGui.inputFloat(labels.get("Time to wiggle"), imFloat));
      eachJointPositionToleranceWidget = new ImFloatWrapper(definition::getEachJointPositionTolerance,
                                                            definition::setEachJointPositionTolerance,
                                                            imFloat -> ImGui.inputFloat(labels.get("Each joint position tolerance (%s)"
                                                                       .formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imFloat));
      sufficientCumulativeJointMovementWidget = new ImFloatWrapper(definition::getSufficientCumulativeJointMovement,
                                                                   definition::setSufficientCumulativeJointMovement,
                                                                   imFloat -> ImGui.inputFloat(labels.get("Sufficient cumulative joint movement (%s)"
                                                                              .formatted(EuclidCoreMissingTools.DEGREE_SYMBOL)), imFloat));

   }

   @Override
   protected void renderImGuiWidgetsInternal()
   {
      ImGui.text("Ability Hand action for " + definition.getSide().getLowerCaseName() + " side");

      ImGui.text("Control Mode:");
      for (AbilityHandControlMode mode : modes)
      {
         if (ImGui.radioButton(labels.get(mode.name()), definition.getControlMode() == mode))
            definition.setControlMode(mode);
         if (mode == AbilityHandControlMode.GRIP)
            ImGui.sameLine();
      }

      if (definition.getControlMode() == AbilityHandControlMode.GRIP)
      {
         if (ImGui.beginCombo(labels.get("Grip"), definition.getGrip().name()))
         {
            for (AbilityHandGrip grip : AbilityHandGrip.values)
            {
               boolean isSelected = definition.getGrip() == grip;
               if (ImGui.selectable(grip.name(), isSelected))
                  definition.setGrip(grip);
               if (isSelected)
                  ImGui.setItemDefaultFocus();
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

      ultimateTimeoutWidget.renderImGuiWidget();

      if (ImGui.beginCombo(labels.get("Success Criteria"), definition.getSuccessCriteria().name()))
         for (SuccessCriteria successCriteria : SuccessCriteria.values)
         {
            boolean selected = definition.getSuccessCriteria() == successCriteria;
            if (ImGui.selectable(successCriteria.name(), selected))
               definition.setSuccessCriteria(successCriteria);
            if (selected)
               ImGui.setItemDefaultFocus();
            ImGui.endCombo();
         }

      switch (definition.getSuccessCriteria())
      {
         case CHECK_EACH_JOINT_POSITION ->
            eachJointPositionToleranceWidget.renderImGuiWidget();
         case CHECK_CUMULATIVE_JOINT_MOVEMENT ->
            sufficientCumulativeJointMovementWidget.renderImGuiWidget();
      }

      enableWiggleOnFailureWidget.renderImGuiWidget();
      if (definition.getEnableWiggleOnFailure())
         timeToWiggleWidget.renderImGuiWidget();
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
