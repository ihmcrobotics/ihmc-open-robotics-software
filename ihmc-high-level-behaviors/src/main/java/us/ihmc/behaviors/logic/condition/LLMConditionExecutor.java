package us.ihmc.behaviors.logic.condition;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.llama.Llama;
import us.ihmc.llamacpp.library.LlamaCPPNativeLibrary;
import us.ihmc.llamacpp.llama_context_params;
import us.ihmc.llamacpp.llama_model_params;
import us.ihmc.llamacpp.llama_sampler;

import static us.ihmc.llamacpp.global.llamacpp.*;

public class LLMConditionExecutor
{
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   public LLMConditionExecutor(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();
   }

   /**
    * For now, we create and dispose everything on each run. This is not quick.
    * FIXME: Allow reusing the model, resetting the context, etc.
    */
   public void updateCurrentlyExecuting()
   {
      LlamaCPPNativeLibrary.load();

      llama_model_params model_params = llama_model_default_params();
      model_params.n_gpu_layers(99);

      llama_context_params ctx_params = llama_context_default_params();
      ctx_params.n_ctx(2048);
      ctx_params.n_batch(2048);

      // initialize the sampler
      llama_sampler smpl = llama_sampler_chain_init(llama_sampler_chain_default_params());
      llama_sampler_chain_add(smpl, llama_sampler_init_min_p(0.05f, 1));
      llama_sampler_chain_add(smpl, llama_sampler_init_temp(0.0f)); // 0 temp important for tests
      llama_sampler_chain_add(smpl, llama_sampler_init_dist(LLAMA_DEFAULT_SEED));

      Llama llama = new Llama(model_params, ctx_params, smpl);

      String prompt = definition.getLLM().getPrompt().getValue();
      state.getLogger().info(prompt);

      String response = llama.generate(prompt);

      state.getLogger().info(response);
      state.setFailed(response.contains("failure"));

      llama.destroy();

      state.setIsExecuting(false); // Completes immediately
   }
}
