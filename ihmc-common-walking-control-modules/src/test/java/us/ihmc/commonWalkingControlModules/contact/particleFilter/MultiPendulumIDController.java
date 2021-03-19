package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class MultiPendulumIDController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   @Override
   public void doControl()
   {

   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return null;
   }
}
