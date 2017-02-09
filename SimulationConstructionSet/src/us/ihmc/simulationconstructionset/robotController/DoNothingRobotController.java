package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class DoNothingRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DoNothing");

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
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
