package us.ihmc.rdx.ui.behavior.logic.condition;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.llama.Llama;
import us.ihmc.rdx.imgui.ImBooleanWrapper;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXLLMCondition
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalBoolean resetContextEachRun;
   private final CRDTBidirectionalBoolean injectBehaviorState;
   private final CRDTBidirectionalBoolean injectEnvironmentState;
   private final CRDTBidirectionalString system;
   private final CRDTBidirectionalString prompt;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper resetContextEachRunCheckbox;
   private final ImBooleanWrapper injectBehaviorStateCheckbox;
   private final ImBooleanWrapper injectEnvironmentStateCheckbox;
   private transient final ImString imSystem = new ImString(Llama.MAX_CONTEXT_SIZE);
   private transient final ImString imPrompt = new ImString(Llama.MAX_CONTEXT_SIZE);

   public RDXLLMCondition(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      resetContextEachRun = definition.getLLM().getResetContextEachRun();
      injectBehaviorState = definition.getLLM().getInjectBehaviorState();
      injectEnvironmentState = definition.getLLM().getInjectEnvironmentState();
      system = definition.getLLM().getSystem();
      prompt = definition.getLLM().getPrompt();

      resetContextEachRunCheckbox = new ImBooleanWrapper(resetContextEachRun::getValue,
                                                         resetContextEachRun::setValue,
                                                         imBoolean -> ImGui.checkbox(labels.get("Reset context before each execution"), imBoolean));
      injectBehaviorStateCheckbox = new ImBooleanWrapper(injectBehaviorState::getValue,
                                                         injectBehaviorState::setValue,
                                                         imBoolean -> ImGui.checkbox(labels.get("Inject behavior state"), imBoolean));
      injectEnvironmentStateCheckbox = new ImBooleanWrapper(injectEnvironmentState::getValue,
                                                            injectEnvironmentState::setValue,
                                                            imBoolean -> ImGui.checkbox(labels.get("Inject environment state"), imBoolean));
   }

   public void renderImGuiWidgetsInternal()
   {
      resetContextEachRunCheckbox.renderImGuiWidget();
      injectBehaviorStateCheckbox.renderImGuiWidget();
      injectEnvironmentStateCheckbox.renderImGuiWidget();

      ImGui.text("System:");
      String systemText = system.getValue();
      String modifiedSystem = ImGuiTools.inputTextMultilineWrap(labels.getHidden("System"), systemText, imSystem);
      if (modifiedSystem != null)
      {
         system.setValue(modifiedSystem);
      }

      ImGui.text("Prompt:");
      String promptText = prompt.getValue();
      String modifiedPrompt = ImGuiTools.inputTextMultilineWrap(labels.getHidden("Prompt"), promptText, imPrompt);
      if (modifiedPrompt != null)
      {
         prompt.setValue(modifiedPrompt);
      }
   }
}
