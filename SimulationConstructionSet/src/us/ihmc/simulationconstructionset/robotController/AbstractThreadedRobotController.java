package us.ihmc.simulationconstructionset.robotController;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.LongYoVariable;
import us.ihmc.simulationconstructionset.Robot;


public abstract class AbstractThreadedRobotController implements RobotController
{
   private final String name;
   protected final YoVariableRegistry registry;

   protected final ArrayList<RobotControllerExecutor> controllers = new ArrayList<RobotControllerExecutor>();
   protected final DoubleYoVariable yoTime;
   protected final Robot simulatedRobot;

   protected final LongYoVariable currentControlTick;

   public AbstractThreadedRobotController(String name, Robot simulatedRobot)
   {
      this.name = name;
      this.registry = new YoVariableRegistry(name);
      this.currentControlTick = new LongYoVariable("currentControlTick", registry);
      this.yoTime = simulatedRobot.getYoTime();
      this.simulatedRobot = simulatedRobot;
   }
   
   public AbstractThreadedRobotController(String name)
   {
      this.name = name;
      this.registry = new YoVariableRegistry(name);
      this.currentControlTick = new LongYoVariable("currentControlTick", registry);
      this.yoTime = new DoubleYoVariable("time", registry);
      this.simulatedRobot = null;
   }

   public abstract void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle);

   public final void initialize()
   {
      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).initialize();
      }
   }

   public final YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public final String getName()
   {
      return name;
   }

   public final String getDescription()
   {
      return getName();
   }

   public void doControl()
   {

      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).waitAndWriteData(currentControlTick.getLongValue());
      }

      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).readData(currentControlTick.getLongValue());
      }
      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).executeForSimulationTick(currentControlTick.getLongValue());
      }
      currentControlTick.increment();
   }

   public void stop()
   {
      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).stop();
      }
      controllers.clear();
   }
}
