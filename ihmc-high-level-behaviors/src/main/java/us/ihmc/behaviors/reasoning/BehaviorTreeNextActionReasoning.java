package us.ihmc.behaviors.reasoning;

import de.kherud.llama.InferenceParameters;
import de.kherud.llama.LlamaModel;
import de.kherud.llama.ModelParameters;
import de.kherud.llama.args.MiroStat;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.log.LogTools;
import us.ihmc.tools.IHMCCommonPaths;

public class BehaviorTreeNextActionReasoning
{
   private static final String SYSTEM = """
         <|start_header_id|>system<|end_header_id|>
         You are a reasoning system that decides the next action to execute in a tree-based robotic system.
         The current tree and state is given by:
         {
           "nodes": [
              {"id": int, "type": string, "children": [ ]}
           ],
           "state": {
             "currently_executing": int
             "is_done": bool
           }
         }
         There are two node types: Action and Sequence.
         An Action node is the only type of node that can be executed.
         A Sequence node can have children. When one child of an action sequence node is done, the next one in the list of children should be executed.
         Please consider which node is best to execute next and output only the node ID number of that action.
         <|eot_id|>
         <|start_header_id|>user<|end_header_id|>
         {
           "nodes": [
              {"id": 001, "type": "Sequence, "children": [
                 {"id": 002, "type": "Action"},
                 {"id": 005, "type": "Action"},
                 {"id": 020, "type": "Action"},
                 {"id": 004, "type": "Action"},
                 {"id": 056, "type": "Action"}
              ]}
           ],
           "state": {
             "currently_executing": 002,
             "is_done": true
           }
         }
         <|eot_id|>
         <|start_header_id|>assistant<|end_header_id|>
         005
         <|eot_id|>
         <|start_header_id|>user<|end_header_id|>
         {
           "nodes": [
              {"id": 001, "type": "Sequence, "children": [
                 {"id": 002, "type": "Action"},
                 {"id": 005, "type": "Action"},
                 {"id": 020, "type": "Action"},
                 {"id": 004, "type": "Action"},
                 {"id": 056, "type": "Action"}
              ]}
           ],
           "state": {
             "currently_executing": 005,
             "is_done": true
           }
         }
         <|eot_id|>
         <|start_header_id|>assistant<|end_header_id|>
         020
         <|eot_id|>
         <|start_header_id|>user<|end_header_id|>
         {
           "nodes": [
              {"id": 001, "type": "Sequence, "children": [
                 {"id": 002, "type": "Action"},
                 {"id": 005, "type": "Action"},
                 {"id": 020, "type": "Action"},
                 {"id": 004, "type": "Action"},
                 {"id": 056, "type": "Action"}
              ]}
           ],
           "state": {
             "currently_executing": 020,
             "is_done": true
           }
         }
         <|eot_id|>
         <|start_header_id|>assistant<|end_header_id|>
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

   public int queryNextLeafToExecuteIndex()
   {
      String prompt = SYSTEM;
//      prompt += """
//            Hello!
//            """;

      InferenceParameters inferParams = new InferenceParameters(prompt);
      inferParams.setPenalizeNl(true);
      inferParams.setTemperature(0.3f);
      inferParams.setMiroStat(MiroStat.V2);
      inferParams.setStopStrings("<|eot_id|>");
      inferParams.setTopK(40);
      inferParams.setTopP(0.25f);
      inferParams.setRepeatPenalty(1.15f);

      String reponse = model.complete(inferParams);

//      LogTools.info(prompt + reponse);
//
//      LogTools.info("Response: {}", reponse);

      return Integer.parseInt(reponse.trim());
   }

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
         int leafIndex = reasoning.queryNextLeafToExecuteIndex();
         LogTools.info("Returned {} in {} seconds", leafIndex, stopwatch.totalElapsed());
      }

      reasoning.destroy();

      System.exit(0); // FIXME: Not sure why it's not exiting automatically.
   }
}
