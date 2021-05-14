package us.ihmc.exampleSimulations.fallingSphere;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class FallingSphereController implements RobotController
{
   private static final long serialVersionUID = -6115066729570319285L;

   private final YoRegistry registry = new YoRegistry("FallingSphereController");


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

   public YoRegistry getYoRegistry()
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
