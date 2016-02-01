package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.robotController.RobotController;


public final class DoNothingController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DoNothingController");

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void doControl()
   {
   }

   public String getName()
   {
      return "doNothing";
   }
   
   public void initialize()
   {      
   }

   public String getDescription()
   {
      return getName();
   }
}