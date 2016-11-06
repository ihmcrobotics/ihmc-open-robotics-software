package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.ArrayList;

import us.ihmc.controlFlow.ControlFlowGraph;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class ControlFlowGraphExecutorController implements RobotController
{
   private final String name = getClass().getSimpleName();

   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ArrayList<ControlFlowGraph> controlFlowGraphs = new ArrayList<ControlFlowGraph>();
   private final ArrayList<Runnable> postRunnables = new ArrayList<Runnable>();
   
   public ControlFlowGraphExecutorController()
   {
   }

   public ControlFlowGraphExecutorController(ControlFlowGraph controlFlowGraph)
   {
      addControlFlowGraph(controlFlowGraph);
   }

   public void addControlFlowGraph(ControlFlowGraph controlFlowGraph)
   {
      this.controlFlowGraphs.add(controlFlowGraph);
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      for(int i = 0; i <  controlFlowGraphs.size(); i++)
      {
         controlFlowGraphs.get(i).startComputation();
      }

      for(int i = 0; i <  controlFlowGraphs.size(); i++)
      {
         controlFlowGraphs.get(i).waitUntilComputationIsDone();
      }
      
      for (int i=0; i<postRunnables.size(); i++)
      {
         postRunnables.get(i).run();
      }
   }

   public void addPostRunnable(Runnable runnable)
   {
      postRunnables.add(runnable);
   }
}
