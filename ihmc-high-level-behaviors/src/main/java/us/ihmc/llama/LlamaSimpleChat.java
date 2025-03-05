package us.ihmc.llama;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.llamacpp.llama_batch;
import us.ihmc.llamacpp.llama_context;
import us.ihmc.llamacpp.llama_context_params;
import us.ihmc.llamacpp.llama_model;
import us.ihmc.llamacpp.llama_model_params;
import us.ihmc.llamacpp.llama_sampler;
import us.ihmc.llamacpp.llama_vocab;
import us.ihmc.log.LogTools;

import static us.ihmc.llamacpp.global.llamacpp.*;

public class LlamaSimpleChat
{
   public static final int MAX_CONTEXT_SIZE = 10000;

   private final llama_model model;
   private final llama_context context;
   private final llama_vocab vocab;
   private final llama_sampler sampler;
   private final StringBuilder contextStringBuilder = new StringBuilder();
   private final StringBuilder responseStringBuilder = new StringBuilder();
   private final Stopwatch stopwatch = new Stopwatch();
   private final IntPointer tokens = new IntPointer(MAX_CONTEXT_SIZE);
   private final BytePointer piece = new BytePointer(256);

   public LlamaSimpleChat()
   {
      llama_model_params modelParameters = llama_model_default_params();
      modelParameters.n_gpu_layers(99);

      llama_context_params contextParameters = llama_context_default_params();
      contextParameters.n_ctx(MAX_CONTEXT_SIZE);
      contextParameters.n_batch(MAX_CONTEXT_SIZE);

      sampler = llama_sampler_chain_init(llama_sampler_chain_default_params());
      llama_sampler_chain_add(sampler, llama_sampler_init_min_p(0.05f, 1));
      llama_sampler_chain_add(sampler, llama_sampler_init_temp(0.0f)); // 0 temp important for tests
      llama_sampler_chain_add(sampler, llama_sampler_init_dist(LLAMA_DEFAULT_SEED));

      model = llama_model_load_from_file(Llama.MODEL_TO_USE.toString(), modelParameters);
      vocab = llama_model_get_vocab(model);
      context = llama_init_from_model(model, contextParameters);

      resetContext();
   }

   public void resetContext()
   {
      contextStringBuilder.delete(0, contextStringBuilder.length());
      contextStringBuilder.append("""
                                  <|start_header_id|>system<|end_header_id|>
                                  %s<|eot_id|>
                                  """.formatted(Llama.CHAT_WITH_LLAMA));
      llama_kv_cache_clear(context);
   }

   public String generate(String request)
   {
      stopwatch.start();

      contextStringBuilder.append("""
                                  <|start_header_id|>user<|end_header_id|>
                                  %s
                                  <|eot_id|>
                                  <|start_header_id|>assistant<|end_header_id|>
                                  """.formatted(request));

      String text = contextStringBuilder.toString();
      boolean isFirst = llama_get_kv_cache_used_cells(context) == 0;
      int numberOfTokens = llama_tokenize(vocab, text, text.length(), tokens, MAX_CONTEXT_SIZE, isFirst, true);
      llama_batch batch = llama_batch_get_one(tokens, numberOfTokens);

      int nextSampledToken;
      responseStringBuilder.delete(0, responseStringBuilder.length());
      while (true)
      {
         // check if we have enough space in the context to evaluate this batch
         int usedKVCells = llama_get_kv_cache_used_cells(context);
         if (usedKVCells + numberOfTokens > MAX_CONTEXT_SIZE)
         {
            LogTools.error("Context size exceeded. usedKVCells: %d + numberOfTokens: %d > MAX_CONTEXT_SIZE: %d"
                                 .formatted(usedKVCells, numberOfTokens, MAX_CONTEXT_SIZE));
            break;
         }

         int result = llama_decode(context, batch);
         if (result > 0)
         {
            LogTools.warn("Could not find a KV slot for the batch (try reducing the size of the batch or increase the context)");
         }
         else if (result < 0)
         {
            LogTools.error("Error decoding: %d. The KV cache state is restored to the state before this call", result);
         }

         nextSampledToken = llama_sampler_sample(sampler, context, -1);

         // is it an end of generation?
         if (llama_vocab_is_eog(vocab, nextSampledToken))
         {
            break;
         }

         // convert the token to a string, print it and add it to the response
         int pieceLength = llama_token_to_piece(vocab, nextSampledToken, piece, (int) piece.capacity(), 0, true);
         if (pieceLength < 0)
         {
            LogTools.error("Failed to convert token to piece");
         }
         piece.limit(pieceLength);
         responseStringBuilder.append(piece.getString());

         // prepare the next batch with the sampled token
         batch.token().put(0, nextSampledToken);
         batch.n_tokens(1);
      }

      String response = responseStringBuilder.toString();

      contextStringBuilder.append("""
                                  %s
                                  <|eot_id|>
                                  """.formatted(response));

      double duration = stopwatch.totalElapsed();
      LogTools.info("Response generation took: %.5f seconds".formatted(duration));

      return response;
   }

   public String getContext()
   {
      return contextStringBuilder.toString();
   }

   public void destroy()
   {
      tokens.close();
      piece.close();
      llama_sampler_free(sampler);
      llama_free(context);
      llama_model_free(model);
   }
}
