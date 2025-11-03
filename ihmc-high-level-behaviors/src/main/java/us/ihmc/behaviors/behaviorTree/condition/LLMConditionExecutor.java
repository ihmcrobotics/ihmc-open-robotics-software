package us.ihmc.behaviors.behaviorTree.condition;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTBidirectionalString;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.llama.Llama;
import us.ihmc.tools.string.StringTools;

import java.util.ArrayList;

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

   private final BehaviorTreeSceneState scene;
   private final BehaviorTreeRootNodeExecutor rootNode;
   private final ConditionNodeState state;
   private final ConditionNodeDefinition definition;

   private final CRDTBidirectionalBoolean resetContextEachRun;
   private final CRDTBidirectionalBoolean injectBehaviorState;
   private final CRDTBidirectionalBoolean injectEnvironmentState;
   private final CRDTBidirectionalString system;
   private final CRDTBidirectionalString prompt;

   public LLMConditionExecutor(ConditionNodeState state, BehaviorTreeRootNodeExecutor rootNode)
   {
      this.state = state;
      this.rootNode = rootNode;
      scene = rootNode.getState().getScene();

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

   public void update()
   {
      if (state.getLLM().pollResetContextRequested())
      {
         state.getLogger().info("Resetting context");
         llama.resetContext(system.getValue());
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

      StringBuilder injectedText = new StringBuilder();
      if (injectBehaviorState.getValue())
      {
         injectedText.append("The following is a description of the current behavior state:\n");
         injectedText.append(BehaviorTreeLLMEncoding.encode(rootNode.getState()) + "\n");
      }
      if (injectEnvironmentState.getValue())
      {
         ArrayList<String> frameNames = new ArrayList<>();
         scene.getAllFrameNames(frameNames::add);

         if (frameNames.isEmpty())
            injectedText.append("There are no currently detected objects in the environment.");
         else
            injectedText.append("The following is a list of poses of things in the environment:\n");

         for (String frameName : frameNames)
         {
            ReferenceFrame frameByName = scene.findFrameByName(frameName);
            injectedText.append(frameByName.getName() + StringTools.tupleString(frameByName.getTransformToWorldFrame().getTranslation())).append("\n");
         }
      }

      promptText = injectedText + "\n" + promptText;
      state.getLogger().info("Prompt: %s".formatted(promptText));

      String response;
      synchronized (lock)
      {
         if (resetContextEachRun.getValue())
            llama.resetContext();

         response = llama.generate(promptText);
      }

      state.getLogger().info(response);

      boolean failure = response.matches(definition.getLLM().getResponseMatcher().getValue());
      boolean matchIsSuccess = definition.getLLM().getMatchIsSuccess().getValue();

      if (matchIsSuccess)
         failure = !failure;

      state.setFailed(failure);

      state.setIsExecuting(false); // Completes immediately
   }
}
