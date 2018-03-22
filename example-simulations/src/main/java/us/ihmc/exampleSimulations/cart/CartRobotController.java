package us.ihmc.exampleSimulations.cart;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CartRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("CartRobotController");

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
   public YoVariableRegistry getYoVariableRegistry()
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
