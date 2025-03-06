package us.ihmc.behaviors.logic.condition;

import us.ihmc.behaviors.logic.ConditionNodeDefinition;
import us.ihmc.behaviors.logic.ConditionNodeState;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.llama.Llama;

/**
 * TODO: Extract context so each node holds it's own
 */
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

   private final CRDTBidirectionalBoolean resetContextEachRun;
   private final CRDTBidirectionalBoolean injectBehaviorState;
   private final CRDTBidirectionalBoolean injectEnvironmentState;
   private final CRDTBidirectionalString system;
   private final CRDTBidirectionalString prompt;

   public LLMConditionExecutor(ConditionNodeState state)
   {
      this.state = state;

      definition = state.getDefinition();

      resetContextEachRun = definition.getLLM().getResetContextEachRun();
      injectBehaviorState = definition.getLLM().getInjectBehaviorState();
      injectEnvironmentState = definition.getLLM().getInjectEnvironmentState();
      system = definition.getLLM().getSystem();
      prompt = definition.getLLM().getPrompt();

      synchronized (lock)
      {
         initialize();
      }
   }

   public void updateCurrentlyExecuting()
   {
      if (!llama.getSystem().equals(system.getValue()))
      {
         state.getLogger().info("Resetting context");
         llama.resetContext(system.getValue());
      }

      String promptText = prompt.getValue();
      state.getLogger().info(promptText);

      String response;
      synchronized (lock)
      {
         if (resetContextEachRun.getValue())
            llama.resetContext();

         if (injectBehaviorState.getValue())
         {
            // TODO
         }
         if (injectEnvironmentState.getValue())
         {
            // TODO
         }

         response = llama.generate(promptText);

         if (response.contains("success"))
         {
            state.getLogger().info("Success, resetting context");
            llama.resetContext();
         }
      }

      state.getLogger().info(response);
      state.setFailed(response.contains("failure"));

      state.setIsExecuting(false); // Completes immediately
   }
}
