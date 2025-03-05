package us.ihmc.behaviors.logic.condition;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.llama.Llama;

public class LLMConditionExecutor
{
   static
   {
      Llama.initialize();
   }

   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;
   private final Llama llama;

   public LLMConditionExecutor(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      llama = new Llama();
   }

   /**
    * For now, we create and dispose everything on each run. This is not quick.
    * FIXME: Allow reusing the model, resetting the context, etc.
    */
   public void updateCurrentlyExecuting()
   {
      String prompt = definition.getLLM().getPrompt().getValue();
      state.getLogger().info(prompt);

      String response = llama.generate(prompt);

      state.getLogger().info(response);
      state.setFailed(response.contains("failure"));

      state.setIsExecuting(false); // Completes immediately
   }

   public void destroy()
   {
      llama.destroy();
   }
}
