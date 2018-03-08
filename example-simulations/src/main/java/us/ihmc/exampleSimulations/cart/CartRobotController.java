package us.ihmc.exampleSimulations.cart;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class CartRobotController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("MobileRobotController");

   private final Robot robot;

   private final PinJoint frontWheelJoint, rearWheelJoint;

   private final double dt;

   
   public CartRobotController(Robot robot, double dt)
   {
      this.robot = robot;
      this.dt = dt;

      frontWheelJoint = (PinJoint) robot.getJoint("frontwheel");
      rearWheelJoint = (PinJoint) robot.getJoint("rearwheel");
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
