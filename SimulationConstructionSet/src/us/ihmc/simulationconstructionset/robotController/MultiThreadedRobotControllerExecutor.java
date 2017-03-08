package us.ihmc.simulationconstructionset.robotController;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotControllerExecutor;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.tools.thread.ThreadTools;

public class MultiThreadedRobotControllerExecutor implements RobotControllerExecutor
{
   private final long ticksPerSimulationTick;
   private final MultiThreadedRobotControlElement robotControlElement;
   private final ExecutorService controlExecutor;

   private final MutableLong currentExecutingTick = new MutableLong();
   private Future<MutableLong> controllerInstance;

   private final MirroredYoVariableRegistry registry;
   private final boolean skipFirstControlCycle;
   
   private final Robot simulatedRobot;

   public MultiThreadedRobotControllerExecutor(Robot simulatedRobot, MultiThreadedRobotControlElement robotControlElement, int ticksPerSimulationTick, boolean skipFirstControlCycle, YoVariableRegistry parentRegistry)
   {
      this.ticksPerSimulationTick = ticksPerSimulationTick;
      this.skipFirstControlCycle = skipFirstControlCycle;
      this.robotControlElement = robotControlElement;
      this.controlExecutor = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(robotControlElement.getName()));
      this.simulatedRobot = simulatedRobot;

      YoVariableRegistry elementRegistry = robotControlElement.getYoVariableRegistry();
      if(elementRegistry!=null)
      {
              this.registry = new MirroredYoVariableRegistry(elementRegistry);
              parentRegistry.addChild(this.registry);
      }
      else
      {
    	  this.registry = null;
      }
   }

   @Override
   public void waitAndWriteData(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         if (controllerInstance != null)
         {
            try
            {
               MutableLong result = controllerInstance.get();

               if (result.val + ticksPerSimulationTick == tick)
               {
                  robotControlElement.write(System.nanoTime());
                  if(registry!=null)
                          registry.updateMirror();
                  updateYoGraphicsListRegistry();
               }
               else
               {
                  System.err.println("Got old control result, simulation state has changed during control.");
                  registry.updateChangedValues();
               }

            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
            catch (ExecutionException e)
            {
               throw new RuntimeException(e);
            }
         }
      }
   }

   @Override
   public void readData(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         robotControlElement.read(System.nanoTime());
      }
   }

   @Override
   public void executeForSimulationTick(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         currentExecutingTick.val = tick;
         controllerInstance = controlExecutor.submit(robotControlElement, currentExecutingTick);
      }
   }

   @Override
   public void initialize()
   {
      robotControlElement.initialize();
   }
   
   @Override
   public long getTicksPerSimulationTick()
   {
      return ticksPerSimulationTick;
   }
   
   private class MutableLong
   {
      long val;
   }
   
   @Override
   public void stop()
   {
      controlExecutor.shutdownNow();
      controllerInstance = null;
   }

   private final RigidBodyTransform transformToWorld = new RigidBodyTransform();

   @Override
   public void updateYoGraphicsListRegistry()
   {
      if(robotControlElement.getYoGraphicsListRegistry() != null)
      {
    	  if(registry!=null)
                 registry.updateChangedValues();
         if (simulatedRobot != null)
         {
            simulatedRobot.getRootJoints().get(0).getTransformToWorld(transformToWorld);
            robotControlElement.getYoGraphicsListRegistry().setSimulationTransformToWorld(transformToWorld);
         }
         robotControlElement.getYoGraphicsListRegistry().update();
      }
   }

}
