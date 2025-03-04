package us.ihmc.rdx.ui.behavior.logic.condition;

import imgui.ImGui;
import imgui.type.ImString;
import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;

public class RDXLLMCondition
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private transient final ImString imPrompt = new ImString(2048);

   public RDXLLMCondition(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();
   }

   public void renderImGuiWidgetsInternal()
   {
      ImGui.text("Prompt:");
      String promptText = definition.getLLM().getPrompt().getValue();

      String modifiedPrompt = ImGuiTools.inputTextMultilineWrap(labels.getHidden("Prompt"), promptText, imPrompt);
      if (modifiedPrompt != null)
      {
         definition.getLLM().getPrompt().setValue(modifiedPrompt);
      }
   }
}
