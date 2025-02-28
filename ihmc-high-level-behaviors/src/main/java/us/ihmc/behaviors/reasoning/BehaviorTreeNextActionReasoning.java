package us.ihmc.behaviors.reasoning;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.llama.Llama;
import us.ihmc.llamacpp.llama_context_params;
import us.ihmc.llamacpp.llama_model_params;
import us.ihmc.llamacpp.llama_sampler;
import us.ihmc.log.LogTools;

import static us.ihmc.llamacpp.global.llamacpp.*;

public class BehaviorTreeNextActionReasoning
{
   private static final String SYSTEM = """
         <|start_header_id|>system<|end_header_id|>
         You are a reasoning system that decides the next action to execute in a tree-based robotic system.
         The following is a schema for how the tree will be represented for a query.
         There is a tree of nodes, where each node's type can be leaf or sequence.
         A sequence node has 0 or more children nodes.
         A leaf node does not have any children.
         The leaves are depth-first ordered and their position in this ordering is given by the index field.
         Each leaf node also has boolean fields for whether it is currently executing, has failed, and can execute.
         The state portion of the scheme gives the global state of the tree.
         The state has a field called execution next index, which is the index of the next node to execute.
         nodes: [
         { type: sequence, children: [
         	{ type: leaf, index: int, is_executing: bool, failed: bool, can_execute: bool } }
         ] } ],
         state: { execution_next_index: int }
         A sequence node defines the order of execution of the children as one after the other.
         The next node to execute should be the one after the last one that is executing.
         If no node's are executing, the next node to execute should remain unchanged.
         Your task is to decide the next left to execute by providing its index.
         <|eot_id|>
         <|start_header_id|>user<|end_header_id|>
         nodes: [
         { type: sequence, children: [
         	{ type: leaf, index: 0, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 1, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 2, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 3, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 4, is_executing: false, failed: false, can_execute: true } }
         ] } ],
         state: { execution_next_index: 0 }
         <|eot_id|>
         <|start_header_id|>assistant<|end_header_id|>
         0
         <|eot_id|>
         <|start_header_id|>user<|end_header_id|>
         nodes: [
         { type: sequence, children: [
         	{ type: leaf, index: 0, is_executing: true, failed: false, can_execute: true } }
         	{ type: leaf, index: 1, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 2, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 3, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 4, is_executing: false, failed: false, can_execute: true } }
         ] } ],
         state: { execution_next_index: 0 }
         <|eot_id|>
         <|start_header_id|>assistant<|end_header_id|>
         1
         <|eot_id|>
         <|start_header_id|>user<|end_header_id|>
         nodes: [
         { type: sequence, children: [
         	{ type: leaf, index: 0, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 1, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 2, is_executing: true, failed: false, can_execute: true } }
         	{ type: leaf, index: 3, is_executing: false, failed: false, can_execute: true } }
         	{ type: leaf, index: 4, is_executing: false, failed: false, can_execute: true } }
         ] } ],
         state: { execution_next_index: 2 }
         <|eot_id|>
         <|start_header_id|>assistant<|end_header_id|>
         3
         <|eot_id|>
         """;

   private final Llama llama;

   public BehaviorTreeNextActionReasoning()
   {
      llama_model_params model_params = llama_model_default_params();
      model_params.n_gpu_layers(33);

      llama_context_params ctx_params = llama_context_default_params();
      ctx_params.n_ctx(2048);
      ctx_params.n_batch(2048);
      ctx_params.n_threads(8);

      llama_sampler smpl = llama_sampler_chain_init(llama_sampler_chain_default_params());
      llama_sampler_chain_add(smpl, llama_sampler_init_min_p(0.05f, 1));
      llama_sampler_chain_add(smpl, llama_sampler_init_temp(0.8f));
      llama_sampler_chain_add(smpl, llama_sampler_init_dist(LLAMA_DEFAULT_SEED));

      llama = new Llama(model_params, ctx_params, smpl);
   }

   public int queryNextLeafToExecuteIndex(BehaviorTreeRootNodeState rootNode)
   {
      String treeEncoding = BehaviorTreeLLMEncoding.encode(rootNode);
      return queryNextLeafToExecuteIndex(treeEncoding);
   }

   public int queryNextLeafToExecuteIndex(String treeEncoding)
   {
      String prompt = SYSTEM;
//      prompt += """
//            <|start_header_id|>user<|end_header_id|>
//            %s
//            <|eot_id|>
//            <|start_header_id|>assistant<|end_header_id|>
//            """.formatted(treeEncoding);


      String reponse = llama.generate("Hello");

      LogTools.info(prompt + reponse);

      return Integer.parseInt(reponse.trim());
   }

   public void destroy()
   {
      llama.destroy();
   }

   public static void main(String[] args)
   {
      BehaviorTreeNextActionReasoning reasoning = new BehaviorTreeNextActionReasoning();

      for  (int i = 0; i < 10; i++)
      {
         Stopwatch stopwatch = new Stopwatch().start();
         int leafIndex = reasoning.queryNextLeafToExecuteIndex("""
            nodes: [
            { type: sequence, children: [
            	{ type: leaf, index: 0, is_executing: false, failed: false, can_execute: true } }
            	{ type: leaf, index: 1, is_executing: false, failed: false, can_execute: true } }
            	{ type: leaf, index: 2, is_executing: false, failed: false, can_execute: true } }
            	{ type: leaf, index: 3, is_executing: true, failed: false, can_execute: true } }
            	{ type: leaf, index: 4, is_executing: false, failed: false, can_execute: true } }
            ] } ],
            state: { execution_next_index: 2 }
            """);
         LogTools.info("Returned {} in {} seconds", leafIndex, stopwatch.totalElapsed());
      }

      reasoning.destroy();

      System.exit(0); // FIXME: Not sure why it's not exiting automatically.
   }
}
