package us.ihmc.rdx.behaviorTree.condition;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeDefinition;
import us.ihmc.behaviors.behaviorTree.condition.ConditionNodeState;
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
   private final CRDTBidirectionalBoolean matchIsSuccess;
   private final CRDTBidirectionalString system;
   private final CRDTBidirectionalString prompt;
   private final CRDTBidirectionalString responseMatcher;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBooleanWrapper resetContextEachRunCheckbox;
   private final ImBooleanWrapper injectBehaviorStateCheckbox;
   private final ImBooleanWrapper injectEnvironmentStateCheckbox;
   private final ImBooleanWrapper matchIsSuccessRadio;
   private transient final ImString imSystem = new ImString(Llama.MAX_CONTEXT_SIZE);
   private transient final ImString imPrompt = new ImString(Llama.MAX_CONTEXT_SIZE);
   private transient final ImString imResponseMatcher = new ImString(Llama.MAX_CONTEXT_SIZE);

   public RDXLLMCondition(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      resetContextEachRun = definition.getLLM().getResetContextEachRun();
      injectBehaviorState = definition.getLLM().getInjectBehaviorState();
      injectEnvironmentState = definition.getLLM().getInjectEnvironmentState();
      matchIsSuccess = definition.getLLM().getMatchIsSuccess();
      system = definition.getLLM().getSystem();
      prompt = definition.getLLM().getPrompt();
      responseMatcher = definition.getLLM().getResponseMatcher();

      resetContextEachRunCheckbox = new ImBooleanWrapper(resetContextEachRun::getValue,
                                                         resetContextEachRun::setValue,
                                                         imBoolean -> ImGui.checkbox(labels.get("Reset context before each execution"), imBoolean));
      injectBehaviorStateCheckbox = new ImBooleanWrapper(injectBehaviorState::getValue,
                                                         injectBehaviorState::setValue,
                                                         imBoolean -> ImGui.checkbox(labels.get("Inject behavior state"), imBoolean));
      injectEnvironmentStateCheckbox = new ImBooleanWrapper(injectEnvironmentState::getValue,
                                                            injectEnvironmentState::setValue,
                                                            imBoolean -> ImGui.checkbox(labels.get("Inject environment state"), imBoolean));
      matchIsSuccessRadio = new ImBooleanWrapper(matchIsSuccess::getValue,
                                                 matchIsSuccess::setValue,
                                                 imBoolean ->
                                                 {
                                                    ImGui.text("Match is:");
                                                    ImGui.sameLine();
                                                    if (ImGui.radioButton(labels.get("success"), imBoolean.get()))
                                                       imBoolean.set(true);
                                                    ImGui.sameLine();
                                                    if (ImGui.radioButton(labels.get("failure"), !imBoolean.get()))
                                                       imBoolean.set(false);
                                                 });
   }

   public void renderImGuiWidgetsInternal()
   {
      resetContextEachRunCheckbox.renderImGuiWidget();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Reset context")))
         state.getLLM().setResetContextRequested();
      injectBehaviorStateCheckbox.renderImGuiWidget();
      injectEnvironmentStateCheckbox.renderImGuiWidget();

      ImGui.text("System:");
      String systemText = system.getValue();
      String modifiedSystem = ImGuiTools.inputTextMultiline(labels.getHidden("System"), systemText, imSystem);
      if (modifiedSystem != null)
      {
         system.setValue(modifiedSystem);
      }

      ImGui.text("Prompt:");
      String promptText = prompt.getValue();
      String modifiedPrompt = ImGuiTools.inputTextMultiline(labels.getHidden("Prompt"), promptText, imPrompt);
      if (modifiedPrompt != null)
      {
         prompt.setValue(modifiedPrompt);
      }

      ImGui.text("Response matcher:");
      matchIsSuccessRadio.renderImGuiWidget();
      String responseMatcherText = responseMatcher.getValue();
      String modifiedResponseMatcher = ImGuiTools.inputTextMultiline(labels.getHidden("Response matcher"), responseMatcherText, imResponseMatcher);
      if (modifiedResponseMatcher != null)
      {
         responseMatcher.setValue(modifiedResponseMatcher);
      }
   }
}
