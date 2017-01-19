package us.ihmc.simulationconstructionset;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;


public final class DoNothingController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("DoNothingController");

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void doControl()
   {
   }

   @Override
   public String getName()
   {
      return "doNothing";
   }
   
   @Override
   public void initialize()
   {      
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}