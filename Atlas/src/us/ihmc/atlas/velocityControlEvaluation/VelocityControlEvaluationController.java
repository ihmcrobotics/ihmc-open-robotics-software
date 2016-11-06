package us.ihmc.atlas.velocityControlEvaluation;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.filters.DelayedDoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class VelocityControlEvaluationController implements RobotController
{
   private final VelocityControlEvaluationRobot robot;
   private final double controlDT;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final DoubleYoVariable q_d_x = new DoubleYoVariable("q_d_x", registry);
   private final DoubleYoVariable qd_d_x = new DoubleYoVariable("qd_d_x", registry);
   private final DoubleYoVariable qdd_d_x = new DoubleYoVariable("qdd_d_x", registry);
   
   private final DoubleYoVariable offset = new DoubleYoVariable("offset", registry);
   private final DoubleYoVariable amplitude = new DoubleYoVariable("amplitude", registry);
   private final DoubleYoVariable frequency = new DoubleYoVariable("frequency", registry);
   
   private final DoubleYoVariable xTrajectory = new DoubleYoVariable("xTrajectory", registry);
   private final DoubleYoVariable xDotTrajectory = new DoubleYoVariable("xDotTrajectory", registry);
   private final DoubleYoVariable xDDotTrajectory = new DoubleYoVariable("xDDotTrajectory", registry);

   private final DoubleYoVariable omega = new DoubleYoVariable("omega", registry);
   private final DoubleYoVariable zeta = new DoubleYoVariable("zeta", registry);
   
   private final DoubleYoVariable kp = new DoubleYoVariable("kp", registry);
   private final DoubleYoVariable kd = new DoubleYoVariable("kd", registry);

   private final DoubleYoVariable alphaDesiredVelocity = new DoubleYoVariable("alphaDesiredVelocity", "Filter for velocity control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking", registry);
   private final DoubleYoVariable kForceVel = new DoubleYoVariable("kForceVel", "Gain for velocity control in order to achieve acceleration control", registry);

   private final DoubleYoVariable qdd_tau = new DoubleYoVariable("qdd_tau", "Torque from inverse dynamics desired acceleration", registry);
   private final DoubleYoVariable qd_tau = new DoubleYoVariable("qd_tau", "Torque from integrating acceleration to get velocity", registry);
   private final DoubleYoVariable undelayedTorque = new DoubleYoVariable("undelayedTorque", "", registry);

   private final DelayedDoubleYoVariable delayedTorque = new DelayedDoubleYoVariable("delayedTorque", "", undelayedTorque, 2, registry);
   
   public VelocityControlEvaluationController(VelocityControlEvaluationRobot robot, double controlDT)
   {
      this.robot = robot;
      this.controlDT = controlDT;
      
      offset.set(0.05);
      amplitude.set(0.2);
      frequency.set(1.0);
      
      omega.set(10.0);
      zeta.set(0.7);
      
      alphaDesiredVelocity.set(0.975);
      
      kForceVel.set(100.0);
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
      return getName();
   }

   public void doControl()
   {
      computeDesiredTrajectory();
      computeGains();
      
      double desiredAcceleration = xDDotTrajectory.getDoubleValue() + kp.getDoubleValue() * (xTrajectory.getDoubleValue() - robot.getX()) + kd.getDoubleValue() * (xDotTrajectory.getDoubleValue() - robot.getXDot());
      qdd_d_x.set(desiredAcceleration);
      
      qd_d_x.set(alphaDesiredVelocity.getDoubleValue() * (qd_d_x.getDoubleValue() + desiredAcceleration * controlDT) + (1.0 - alphaDesiredVelocity.getDoubleValue()) * robot.getXDot()) ;
      
      qdd_tau.set(qdd_d_x.getDoubleValue() * VelocityControlEvaluationRobot.MASS);
      qd_tau.set(kForceVel.getDoubleValue() * (qd_d_x.getDoubleValue() - robot.getXDot()));
      
      undelayedTorque.set(qdd_tau.getDoubleValue() + qd_tau.getDoubleValue());
//      undelayedTorque.set(qd_tau.getDoubleValue());
      delayedTorque.update();
      
      robot.setTau(delayedTorque.getDoubleValue());
   }

   private void computeGains()
   {
      kp.set(omega.getDoubleValue() * omega.getDoubleValue());
      kd.set(2.0 * zeta.getDoubleValue() * omega.getDoubleValue());
   }

   private void computeDesiredTrajectory()
   {
      double twoPiFreq = 2.0 * Math.PI * frequency.getDoubleValue();
      
      xTrajectory.set(offset.getDoubleValue() + amplitude.getDoubleValue() * Math.sin(twoPiFreq * robot.getTime()));
      xDotTrajectory.set(twoPiFreq * amplitude.getDoubleValue() * Math.cos(twoPiFreq * robot.getTime()));
      xDDotTrajectory.set(-twoPiFreq * twoPiFreq * amplitude.getDoubleValue() * Math.sin(twoPiFreq * robot.getTime()));
   }

}
