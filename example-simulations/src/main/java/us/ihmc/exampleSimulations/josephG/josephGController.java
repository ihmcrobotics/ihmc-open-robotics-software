package us.ihmc.exampleSimulations.josephG;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class josephGController implements RobotController
{
   // A name for this controller
   private final String name = "josephGController";
   
   // This line instantiates a registry that will contain relevant controller variables that will be accessible from the simulation panel.
   private final YoRegistry registry = new YoRegistry("josephGController");
   
   private josephGRobot robot;
   
   public josephGController(josephGRobot robot)
   {
      this.robot = robot;
   }
   
   @Override public void initialize()
   {
      
   }
   
   @Override public void doControl()
   {
      /*START HERE.... FIGURE OUT HOW TO CHECK FOR ANGLE POSITION*/
   }
   
   @Override public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
