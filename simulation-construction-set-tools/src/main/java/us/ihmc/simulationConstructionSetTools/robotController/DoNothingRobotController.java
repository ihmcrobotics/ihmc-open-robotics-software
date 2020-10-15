package us.ihmc.simulationConstructionSetTools.robotController;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DoNothingRobotController implements RobotController
{
   private final YoRegistry registry = new YoRegistry("DoNothing");

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
      return "DoNothing";
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
   }

}
