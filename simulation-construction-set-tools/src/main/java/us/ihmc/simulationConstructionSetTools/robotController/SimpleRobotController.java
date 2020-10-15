package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class SimpleRobotController implements RobotController
{
   private final String name = getClass().getSimpleName();
   protected final YoRegistry registry = new YoRegistry(name);

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }

   @Override
   abstract public void doControl();
}
