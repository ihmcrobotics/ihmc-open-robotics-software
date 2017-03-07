package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotControllerExecutor;
import us.ihmc.simulationconstructionset.Robot;

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

   @Override
   public void waitAndWriteData(long tick)
   {
      if (tick % ticksPerSimulationTick == 0 && !(tick == 0 && skipFirstControlCycle))
      {
         robotControlElement.write(System.nanoTime());
         if (simulatedRobot != null)
         {
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            simulatedRobot.getRootJoints().get(0).getTransformToWorld(transformToWorld);
            if (robotControlElement.getYoGraphicsListRegistry() != null)
            {
               robotControlElement.getYoGraphicsListRegistry().setSimulationTransformToWorld(transformToWorld);
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
         robotControlElement.run();
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

   @Override
   public void stop()
   {
      // Nothing to do here.
   }

   @Override
   public void updateYoGraphicsListRegistry()
   {
   }
   

}
