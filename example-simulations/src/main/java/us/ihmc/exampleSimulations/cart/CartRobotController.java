package us.ihmc.exampleSimulations.cart;

import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CartRobotController implements RobotController
{
   private final YoRegistry registry = new YoRegistry("CartRobotController");

   private final PinJoint frontWheelJoint;

   public CartRobotController(Robot robot)
   {
      frontWheelJoint = (PinJoint) robot.getJoint("frontWheel");
   }

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
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      frontWheelJoint.setTau(50.0);
   }
}
