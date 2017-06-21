package us.ihmc.simulationConstructionSetTools.robotController;

import java.util.ArrayList;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotController.RobotControllerExecutor;
import us.ihmc.simulationconstructionset.Robot;

public abstract class AbstractThreadedRobotController implements RobotController
{
   private final String name;
   protected final YoVariableRegistry registry;

   protected final ArrayList<RobotControllerExecutor> controllers = new ArrayList<RobotControllerExecutor>();
   protected final YoDouble yoTime;
   protected final Robot simulatedRobot;

   protected final YoLong currentControlTick;

   public AbstractThreadedRobotController(String name, Robot simulatedRobot)
   {
      this.name = name;
      this.registry = new YoVariableRegistry(name);
      this.currentControlTick = new YoLong("currentControlTick", registry);
      this.yoTime = simulatedRobot.getYoTime();
      this.simulatedRobot = simulatedRobot;
   }
   
   public AbstractThreadedRobotController(String name)
   {
      this.name = name;
      this.registry = new YoVariableRegistry(name);
      this.currentControlTick = new YoLong("currentControlTick", registry);
      this.yoTime = new YoDouble("time", registry);
      this.simulatedRobot = null;
   }

   public abstract void addController(MultiThreadedRobotControlElement controller, int executionsPerControlTick, boolean skipFirstControlCycle);

   @Override
   public final void initialize()
   {
      for (int i = 0; i < controllers.size(); i++)
      {
         controllers.get(i).initialize();
      }
   }

   @Override
   public final YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public final String getName()
   {
      return name;
   }

   @Override
   public final String getDescription()
   {
      return getName();
   }

   @Override
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
