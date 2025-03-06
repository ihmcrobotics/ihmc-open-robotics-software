package us.ihmc.behaviors.logic.condition;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.llama.Llama;

public class LLMConditionExecutor
{
   private static final Object lock = new Object();
   private static Llama llama;
   private static boolean initialized = false;
   private static boolean destroyed = false;

   public static void initialize()
   {
      if (!initialized)
      {
         initialized = true;
         Llama.initialize();
         llama = new Llama();
      }
   }

   public static void destroy()
   {
      synchronized (lock)
      {
         if (initialized && !destroyed)
         {
            destroyed = true;
            llama.destroy();
         }
      }
   }

   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   public LLMConditionExecutor(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      synchronized (lock)
      {
         initialize();
      }
   }

   public void updateCurrentlyExecuting()
   {
      String prompt = definition.getLLM().getPrompt().getValue();
      state.getLogger().info(prompt);

      String response;
      synchronized (lock)
      {
         response = llama.generate(prompt);
      }

      state.getLogger().info(response);
      state.setFailed(response.contains("failure"));

      state.setIsExecuting(false); // Completes immediately
   }
}
