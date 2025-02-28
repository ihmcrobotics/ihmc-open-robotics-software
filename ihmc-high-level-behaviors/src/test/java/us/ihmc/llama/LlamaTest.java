package us.ihmc.llama;

import org.junit.jupiter.api.Test;
import us.ihmc.llamacpp.library.LlamaCPPNativeLibrary;
import us.ihmc.llamacpp.llama_context_params;
import us.ihmc.llamacpp.llama_model_params;
import us.ihmc.llamacpp.llama_sampler;
import us.ihmc.log.LogTools;

import static us.ihmc.llamacpp.global.llamacpp.*;
import static us.ihmc.llamacpp.global.llamacpp.LLAMA_DEFAULT_SEED;

public class LlamaTest
{
   @Test
   public void testLlama()
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

      String response;
      response = llama.generate("What is 2 + 2?");
      LogTools.info(response);
      response = llama.generate("What is 5 + 8?");
      LogTools.info(response);
      response = llama.generate("What is the capital of the USA?");
      LogTools.info(response);

//      llama.clearContext();

      response = llama.generate("There are 3 fruit, a banana, an apple, and a pear. Which fruit is likely to be red?");
      LogTools.info(response);
      response = llama.generate("List the colors of the other ones.");
      LogTools.info(response);
//
//      llama.clearContext();
      response = llama.generate("List the fruit we just discussed.");
      LogTools.info(response);

      llama.destroy();
   }
}
