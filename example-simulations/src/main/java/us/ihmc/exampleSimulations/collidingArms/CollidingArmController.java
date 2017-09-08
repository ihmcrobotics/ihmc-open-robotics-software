package us.ihmc.exampleSimulations.collidingArms;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class CollidingArmController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("CollidingArmController");

   private final YoDouble qd_d_base = new YoDouble("qd_d_base", registry);

   private final YoDouble q_d_base = new YoDouble("q_d_base", registry);
   private final YoDouble q_d_shoulder = new YoDouble("q_d_shoulder", registry);
   private final YoDouble q_d_elbow = new YoDouble("q_d_elbow", registry);

   private final YoDouble kp_base = new YoDouble("kp_base", registry);
   private final YoDouble kp_shoulder = new YoDouble("kp_shoulder", registry);
   private final YoDouble kp_elbow = new YoDouble("kp_elbow", registry);

   private final YoDouble kd_base = new YoDouble("kd_base", registry);
   private final YoDouble kd_shoulder = new YoDouble("kd_shoulder", registry);
   private final YoDouble kd_elbow = new YoDouble("kd_elbow", registry);

   private final Robot robot;

   private final PinJoint baseJoint, shoulderJoint, elbowJoint;

   private final double dt;

   public CollidingArmController(Robot robot, double dt)
   {
      this.robot = robot;
      this.dt = dt;

      baseJoint = (PinJoint) robot.getRootJoints().get(0);
      shoulderJoint = (PinJoint) baseJoint.getChildrenJoints().get(0);
      elbowJoint = (PinJoint) shoulderJoint.getChildrenJoints().get(0);

      qd_d_base.set(5.0);
      
      q_d_base.set(0.0);
      q_d_shoulder.set(0.4);
      q_d_elbow.set(2.4);

      kp_base.set(100.0);
      kp_shoulder.set(200.0);
      kp_elbow.set(100.0);

      kd_base.set(30.0);
      kd_shoulder.set(30.0);
      kd_elbow.set(30.0);
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
      q_d_base.set(q_d_base.getDoubleValue() + qd_d_base.getDoubleValue() * dt);

      double tauBase = kp_base.getDoubleValue() * (q_d_base.getDoubleValue() - baseJoint.getQ()) - kd_base.getDoubleValue() * baseJoint.getQD();
      double tauShoulder = kp_shoulder.getDoubleValue() * (q_d_shoulder.getDoubleValue() - shoulderJoint.getQ()) - kd_shoulder.getDoubleValue() * shoulderJoint.getQD();
      double tauElbow = kp_elbow.getDoubleValue() * (q_d_elbow.getDoubleValue() - elbowJoint.getQ()) - kd_elbow.getDoubleValue() * elbowJoint.getQD();
      
      baseJoint.setTau(tauBase);
      shoulderJoint.setTau(tauShoulder);
      elbowJoint.setTau(tauElbow);
   }

}
