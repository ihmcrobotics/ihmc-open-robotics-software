package us.ihmc.robotics.robotController;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * Designed for convenience in tests if you don't want to implement everything.
 */
public class RobotControllerAdapter implements RobotController
{
   private final YoRegistry registry;
   
   public RobotControllerAdapter(YoRegistry registry)
   {
      this.registry = registry;
   }
   
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
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void doControl()
   {
   }
}
