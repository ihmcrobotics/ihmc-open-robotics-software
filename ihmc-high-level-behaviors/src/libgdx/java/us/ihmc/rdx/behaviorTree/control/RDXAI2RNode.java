package us.ihmc.rdx.behaviorTree.control;

import imgui.ImGui;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeDefinition;
import us.ihmc.behaviors.behaviorTree.control.ai2r.AI2RNodeState;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeNode;
import us.ihmc.rdx.behaviorTree.RDXBehaviorTreeRootNode;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImDoubleWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImIntegerWrapper;

public class RDXAI2RNode extends RDXBehaviorTreeNode<AI2RNodeState, AI2RNodeDefinition>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper randomizeGoToActionCheckbox;
   private final ImIntegerWrapper numberOfRandomizationsInput;
   private final ImBooleanWrapper randomizeWholeBodyActionCheckbox;
   private final ImDoubleWrapper probabilityRightArmInput;
   private final ImDoubleWrapper probabilityLeftArmInput;
   private final ImDoubleWrapper probabilityPelvisInput;
   private final ImDoubleWrapper probabilityTorsoInput;

   public RDXAI2RNode(long id, RDXBehaviorTreeRootNode rootNode)
   {
      super(new AI2RNodeState(id, rootNode.getState()), rootNode);

      randomizeGoToActionCheckbox = new ImBooleanWrapper(definition::getRandomizeGoToActionEnabled,
                                                         definition::setRandomizeGoToActionEnabled,
                                                         imBoolean -> ImGui.checkbox(labels.get("Randomize Go To Action"), imBoolean));
      numberOfRandomizationsInput = new ImIntegerWrapper(definition::getNumberOfRandomizationsValue,
                                                         definition::setNumberOfRandomizationsValue,
                                                         imInt -> ImGuiTools.volatileInputInt(labels.get("Number of Randomizations"), imInt, 1));
      randomizeWholeBodyActionCheckbox = new ImBooleanWrapper(definition::getRandomizeWholeBodyActionEnabled,
                                                              definition::setRandomizeWholeBodyActionEnabled,
                                                              imBoolean -> ImGui.checkbox(labels.get("Randomize Whole Body Action"), imBoolean));
      probabilityRightArmInput = new ImDoubleWrapper(definition::getProbabilityRightArmEnabled,
                                                     definition::setProbabilityRightArmEnabled,
                                                     imDouble -> ImGui.inputDouble(labels.get("Prob Right Arm Enabled"), imDouble));
      probabilityLeftArmInput = new ImDoubleWrapper(definition::getProbabilityLeftArmEnabled,
                                                    definition::setProbabilityLeftArmEnabled,
                                                    imDouble -> ImGui.inputDouble(labels.get("Prob Left Arm Enabled"), imDouble));
      probabilityPelvisInput = new ImDoubleWrapper(definition::getProbabilityPelvisEnabled,
                                                   definition::setProbabilityPelvisEnabled,
                                                   imDouble -> ImGui.inputDouble(labels.get("Prob Pelvis Enabled"), imDouble));
      probabilityTorsoInput = new ImDoubleWrapper(definition::getProbabilityTorsoEnabled,
                                                  definition::setProbabilityTorsoEnabled,
                                                  imDouble -> ImGui.inputDouble(labels.get("Prob Torso Enabled"), imDouble));
   }

   @Override
   public void renderNodeSettingsWidgets()
   {
      randomizeGoToActionCheckbox.renderImGuiWidget();
      if (randomizeGoToActionCheckbox.changed())
         definition.modify();

      numberOfRandomizationsInput.renderImGuiWidget();
      if (numberOfRandomizationsInput.changed())
         definition.modify();

      ImGui.separator();
      randomizeWholeBodyActionCheckbox.renderImGuiWidget();

      probabilityRightArmInput.renderImGuiWidget();
      probabilityLeftArmInput.renderImGuiWidget();
      probabilityPelvisInput.renderImGuiWidget();
      probabilityTorsoInput.renderImGuiWidget();

      if (randomizeWholeBodyActionCheckbox.changed()
          || probabilityRightArmInput.changed() || probabilityLeftArmInput.changed()
          || probabilityPelvisInput.changed() || probabilityTorsoInput.changed())
      {
         definition.modify();
      }

      super.renderNodeSettingsWidgets();
   }
}
