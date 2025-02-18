package us.ihmc.behaviors.reasoning;

import de.kherud.llama.InferenceParameters;
import de.kherud.llama.LlamaModel;
import de.kherud.llama.ModelParameters;
import de.kherud.llama.args.MiroStat;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

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


   private final LlamaModel model;

   public BehaviorTreeNextActionReasoning()
   {
      String modelFilePath = IHMCCommonPaths.DOT_IHMC_DIRECTORY.resolve("llama-models/Llama-3.2-1B-Instruct-Q8_0.gguf").toString();
      ModelParameters modelParams = new ModelParameters();
      modelParams.setModelFilePath(modelFilePath);
      modelParams.setNGpuLayers(33);
      modelParams.setNThreads(8);
      modelParams.setNCtx(4098);

      LlamaModel.setLogger(null, (level, message) -> {});

      model = new LlamaModel(modelParams);
   }

   public int queryNextLeafToExecuteIndex(BehaviorTreeRootNodeState rootNode)
   {
      String treeEncoding = BehaviorTreeLLMEncoding.encode(rootNode);
      return queryNextLeafToExecuteIndex(treeEncoding);
   }

   public int queryNextLeafToExecuteIndex(String treeEncoding)
   {
      String prompt = SYSTEM;
      prompt += """
            <|start_header_id|>user<|end_header_id|>
            %s
            <|eot_id|>
            <|start_header_id|>assistant<|end_header_id|>
            """.formatted(treeEncoding);

      InferenceParameters inferParams = new InferenceParameters(prompt);
      inferParams.setPenalizeNl(true);
      inferParams.setTemperature(0.3f);
      inferParams.setMiroStat(MiroStat.V2);
      inferParams.setStopStrings("<|eot_id|>");
      inferParams.setTopK(40);
      inferParams.setTopP(0.25f);
      inferParams.setRepeatPenalty(1.15f);

      String reponse = model.complete(inferParams);

      LogTools.info(prompt + reponse);

      return Integer.parseInt(reponse.trim());
   }

   // FIXME: Doesn't work yet
   public void destroy()
   {
      model.close();
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
