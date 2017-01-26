package us.ihmc.exampleSimulations.fallingSphere;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotController.RobotController;

public class FallingSphereController implements RobotController
{
   private static final long serialVersionUID = -6115066729570319285L;

   private final YoVariableRegistry registry = new YoVariableRegistry("FallingSphereController");


   private final FallingSphereRobot robot;
   
   public FallingSphereController(FallingSphereRobot robot)
   {
      this.robot = robot;
   }
   

   public void doControl()
   {
      robot.computeEnergy();
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getDescription()
   {
      return registry.getName();
   }

   public String getName()
   {
      return registry.getName();
   }

}
