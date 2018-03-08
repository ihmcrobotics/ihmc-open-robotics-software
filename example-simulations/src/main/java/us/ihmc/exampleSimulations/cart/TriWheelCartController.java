package us.ihmc.exampleSimulations.cart;

import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class TriWheelCartController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("MobileRobotController");

   private final Robot robot;

   private final PinJoint leftWheelJoint, rightWheelJoint, casterAxisJoint;

   private double t;
   private final double dt;
   
   public TriWheelCartController(Robot robot, double dt)
   {
      this.robot = robot;
      this.dt = dt;

      leftWheelJoint = (PinJoint) robot.getJoint("leftwheel");
      rightWheelJoint = (PinJoint) robot.getJoint("rightwheel");
      casterAxisJoint = (PinJoint) robot.getJoint("casteraxis");
      
      casterAxisJoint.setInitialState(0.5*Math.PI, 0.0);
      t = 0;
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
      t = t + dt;
      
      leftWheelJoint.setTau(50.0 * Math.sin(t/4.0*Math.PI*2));
      rightWheelJoint.setTau(50.0 * Math.cos(t/7.0*Math.PI*2));
   }
}
