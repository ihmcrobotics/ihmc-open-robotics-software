package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;

class SingleThreadedRobotControllerExecutor implements RobotControllerExecutor
{
   private final long ticksPerSimulationTick;
   private final MultiThreadedRobotControlElement robotControlElement;
   private final boolean skipFirstControlCycle;
   private final Robot simulatedRobot;
   
   SingleThreadedRobotControllerExecutor(Robot simulatedRobot, MultiThreadedRobotControlElement robotControlElement, int ticksPerSimulationTick, boolean skipFirstControlCycle, YoVariableRegistry parentRegistry)
   {
      this.ticksPerSimulationTick = ticksPerSimulationTick;
      this.robotControlElement = robotControlElement;
      this.skipFirstControlCycle = skipFirstControlCycle;
      this.simulatedRobot = simulatedRobot;
      
      parentRegistry.addChild(robotControlElement.getYoVariableRegistry());
   }

   public void waitAndWriteData(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         robotControlElement.write(System.nanoTime());
         if (simulatedRobot != null)
         {
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            simulatedRobot.getRootJoints().get(0).getTransformToWorld(transformToWorld);
            if (robotControlElement.getDynamicGraphicObjectsListRegistry() != null)
            {
               robotControlElement.getDynamicGraphicObjectsListRegistry().setSimulationTransformToWorld(transformToWorld);
            }
         }
      }
   }

   public void readData(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         robotControlElement.read(System.nanoTime());
      }
   }

   public void executeForSimulationTick(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         robotControlElement.run();
      }
   }

   public void initialize()
   {
      robotControlElement.initialize();
   }
   
   public long getTicksPerSimulationTick()
   {
      return ticksPerSimulationTick;
   }

   public void stop()
   {
      // Nothing to do here.
   }

   public void updateDynamicGraphicObjectListRegistry()
   {
   }
   

}
