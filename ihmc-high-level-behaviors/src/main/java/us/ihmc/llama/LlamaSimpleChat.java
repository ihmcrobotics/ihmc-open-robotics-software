package us.ihmc.llama;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.javacpp.Pointer;
import org.junit.jupiter.api.Disabled;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.llamacpp.llama_batch;
import us.ihmc.llamacpp.llama_chat_message;
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
   private llama_model model;
   private llama_context context;
   private llama_vocab vocab;
   private StringBuilder contextStringBuilder = new StringBuilder();
   private BytePointer contextStringPointer;
   private final Stopwatch stopwatch = new Stopwatch();

   public LlamaSimpleChat()
   {
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

      model = llama_model_load_from_file(Llama.MODEL_TO_USE.toString(), model_params);
      vocab = llama_model_get_vocab(model);
      context = llama_init_from_model(model, ctx_params);
      contextStringPointer = new BytePointer(llama_n_ctx(context));

      resetContext();
   }

   public void resetContext()
   {
      contextStringBuilder.append("""
                                  <|start_header_id|>system<|end_header_id|>
                                  %s
                                  <|eot_id|>
                                  """.formatted(Llama.CHAT_WITH_LLAMA));
   }

   public String generate(String request)
   {
      stopwatch.start();

      String prompt = contextStringBuilder.toString();

      StringBuilder response_builder = new StringBuilder();

      boolean is_first = llama_get_kv_cache_used_cells(context) == 0;

      int n_prompt_tokens = -llama_tokenize(vocab, prompt, prompt.length(), (IntPointer) null, 0, is_first, true);
      IntPointer prompt_tokens = new IntPointer(n_prompt_tokens);
      if (llama_tokenize(vocab, prompt, prompt.length(), prompt_tokens, n_prompt_tokens, is_first, true) < 0)
         LogTools.error("Failed to tokenize the prompt");

      // prepare a batch for the prompt
      llama_batch batch = llama_batch_get_one(prompt_tokens, n_prompt_tokens);

      int new_token_id;
      while (true)
      {
         // check if we have enough space in the context to evaluate this batch
         int n_ctx = llama_n_ctx(context);
         int n_ctx_used = llama_get_kv_cache_used_cells(context);
         if (n_ctx_used + batch.n_tokens() > n_ctx)
         {
            LogTools.error("Context size exceeded");
            break;
         }

         if (llama_decode(context, batch) != 0)
            LogTools.error("Failed to decode");

         // sample the next token
         new_token_id = llama_sampler_sample(smpl, context, -1);

         // is it an end of generation?
         if (llama_vocab_is_eog(vocab, new_token_id))
         {
            break;
         }

         // convert the token to a string, print it and add it to the response
         byte[] buf = new byte[256];
         int n = llama_token_to_piece(vocab, new_token_id, buf, buf.length, 0, true);
         if (n < 0)
         {
            LogTools.error("Failed to convert token to piece");
         }
         String piece = new String(buf, 0, n);
         response_builder.append(piece);

         // prepare the next batch with the sampled token
         batch.token().put(0, new_token_id);
         batch.n_tokens(1);
      }

      String response = response_builder.toString();

      // add the response to the messages
      push_back_message("assistant", response);
      prev_len = llama_chat_apply_template(tmpl, messages, n_messages, false, (BytePointer) null, 0);
      if (prev_len < 0)
      {
         LogTools.error("Failed to apply the chat template");
      }

      double duration = stopwatch.totalElapsed();
      LogTools.info("Response generation took: %.5f seconds".formatted(duration));

      return response;
   }

   @Disabled // FIXME: Not working yet
   public void addMessage(String role, String content)
   {
      String tmpl = llama_model_chat_template(model, (String) null);

      // add the user input to the message list and format it
      push_back_message(role, content);
      int new_len = llama_chat_apply_template(tmpl, messages, n_messages, false, contextStringPointer, (int) contextStringPointer.capacity());
      if (new_len > contextStringPointer.capacity())
      {
         contextStringPointer = new BytePointer(new_len);
         new_len = llama_chat_apply_template(tmpl, messages, n_messages, false, contextStringPointer, (int) contextStringPointer.capacity());
      }
      if (new_len < 0)
      {
         LogTools.error("Failed to apply the chat template");
      }
   }

   private void push_back_message(String role, String content)
   {
      if (messages.capacity() == n_messages)
      {
         LogTools.info("Allocating new messages");
         llama_chat_message messages_new = new llama_chat_message((long) n_messages * 2);
         for (int i = 0; i < n_messages; i++)
            Pointer.memcpy(messages_new, messages, n_messages);
         messages.close();
         messages = messages_new;
      }

      llama_chat_message message = messages.getPointer(n_messages++);
      message.role(new BytePointer(role));
      message.content(new BytePointer(content));
   }

   @Deprecated // FIXME: Not working yet
   public void clearContext()
   {
      contextStringPointer.close();
      messages.close();
      llama_sampler_free(smpl);
      llama_free(context);
      llama_model_free(model);

      model = llama_model_load_from_file(MODEL_TO_USE.toString(), model_params);
      vocab = llama_model_get_vocab(model);
      context = llama_init_from_model(model, ctx_params);

      contextStringPointer = new BytePointer(llama_n_ctx(context));
      prev_len = 0;
      messages = new llama_chat_message(100);
      n_messages = 0;
   }

   public String getContext()
   {
      return contextStringPointer.getString();
   }

   public void destroy()
   {
      // free resources
      contextStringPointer.close();
      messages.close();
      llama_sampler_free(smpl);
      llama_free(context);
      llama_model_free(model);
   }
}
