package us.ihmc.sensorProcessing.stateEstimation.evaluation;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class RunnableRunnerController implements RobotController
{
   private final String name = getClass().getSimpleName();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final ArrayList<Runnable> runnables = new ArrayList<Runnable>();

   public RunnableRunnerController()
   {
   }

   public RunnableRunnerController(Runnable runnable)
   {
      addRunnable(runnable);
   }

   public void addRunnable(Runnable runnable)
   {
      this.runnables.add(runnable);
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
      for(int i = 0; i <  runnables.size(); i++)
      {
         runnables.get(i).run();
      }
   }
}
