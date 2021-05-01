package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePendulumController implements RobotController
{
   private static final double maxTau = 1000;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DoublePendulumRobot robot;

   private final YoDouble jointStiffness = new YoDouble("jointStiffness", registry);
   private final YoDouble jointDamping = new YoDouble("jointDamping", registry);

   private final YoDouble joint1Setpoint = new YoDouble("joint1Setpoint", registry);
   private final YoDouble joint2Setpoint = new YoDouble("joint2Setpoint", registry);

   public DoublePendulumController(DoublePendulumRobot robot)
   {
      this.robot = robot;
      jointStiffness.set(350.0);
      jointDamping.set(30.0);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public void initialize()
   {
   }

   public void setSetpoints(double q1, double q2)
   {
      joint1Setpoint.set(q1);
      joint2Setpoint.set(q2);
   }

   @Override
   public void doControl()
   {
      double deltaQ1 = AngleTools.computeAngleDifferenceMinusPiToPi(joint1Setpoint.getValue(), robot.getScsJoint1().getQ());
      double deltaQ2 = AngleTools.computeAngleDifferenceMinusPiToPi(joint2Setpoint.getValue(), robot.getScsJoint2().getQ());

      double tau1 = jointStiffness.getDoubleValue() * deltaQ1 - jointDamping.getValue() * (robot.getScsJoint1().getQD());
      double tau2 = jointStiffness.getDoubleValue() * deltaQ2 - jointDamping.getValue() * (robot.getScsJoint2().getQD());

      robot.getScsJoint1().setTau(MathTools.clamp(tau1, maxTau));
      robot.getScsJoint2().setTau(MathTools.clamp(tau2, maxTau));

      robot.setIDStateFromSCS();
   }
}
