package us.ihmc.exampleSimulations.fallingCylinder;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FallingCylinderController implements RobotController
{
   private final String name = "fallingCylinderController";
   
   private final YoRegistry registry = new YoRegistry("fallingCylinderController");
   
   private FallingCylinderRobot robot;
   
   /* Control Variables */
   
   
   public FallingCylinderController(FallingCylinderRobot robot)
   {
      this.robot = robot;
      
   }
   
   @Override
   public void initialize()
   {
      // TODO Auto-generated method stub
      
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void doControl()
   {
      // TODO Auto-generated method stub
      
   }
   
}
