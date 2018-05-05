package us.ihmc.exampleSimulations.cart;

import us.ihmc.simulationConstructionSet.util.RobotController;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class TriWheelCartController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("WheelCartRobotController");
   
   private final PinJoint leftWheelJoint, rightWheelJoint, casterAxisJoint;

   private final YoDouble yoTime;

   public TriWheelCartController(Robot robot)
   {
      leftWheelJoint = (PinJoint) robot.getJoint("leftWheel");
      rightWheelJoint = (PinJoint) robot.getJoint("rightWheel");
      casterAxisJoint = (PinJoint) robot.getJoint("casterAxis");

      yoTime = robot.getYoTime();

      casterAxisJoint.setInitialState(0.5 * Math.PI, 0.0);
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
      leftWheelJoint.setTau(50.0 * Math.sin(yoTime.getDoubleValue() / 4.0 * Math.PI * 2));
      rightWheelJoint.setTau(50.0 * Math.cos(yoTime.getDoubleValue() / 7.0 * Math.PI * 2));
   }
}
