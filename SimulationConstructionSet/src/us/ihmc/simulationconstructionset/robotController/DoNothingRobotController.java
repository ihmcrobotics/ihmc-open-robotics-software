package us.ihmc.simulationconstructionset.robotController;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;

public class DoNothingRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DoNothing");

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return "DoNothing";
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
   }

}
