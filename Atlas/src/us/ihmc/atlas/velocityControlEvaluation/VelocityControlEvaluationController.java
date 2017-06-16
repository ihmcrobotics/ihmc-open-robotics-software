package us.ihmc.atlas.velocityControlEvaluation;

import us.ihmc.robotics.math.filters.DelayedYoDouble;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.robotController.RobotController;

public class VelocityControlEvaluationController implements RobotController
{
   private final VelocityControlEvaluationRobot robot;
   private final double controlDT;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble q_d_x = new YoDouble("q_d_x", registry);
   private final YoDouble qd_d_x = new YoDouble("qd_d_x", registry);
   private final YoDouble qdd_d_x = new YoDouble("qdd_d_x", registry);
   
   private final YoDouble offset = new YoDouble("offset", registry);
   private final YoDouble amplitude = new YoDouble("amplitude", registry);
   private final YoDouble frequency = new YoDouble("frequency", registry);
   
   private final YoDouble xTrajectory = new YoDouble("xTrajectory", registry);
   private final YoDouble xDotTrajectory = new YoDouble("xDotTrajectory", registry);
   private final YoDouble xDDotTrajectory = new YoDouble("xDDotTrajectory", registry);

   private final YoDouble omega = new YoDouble("omega", registry);
   private final YoDouble zeta = new YoDouble("zeta", registry);
   
   private final YoDouble kp = new YoDouble("kp", registry);
   private final YoDouble kd = new YoDouble("kd", registry);

   private final YoDouble alphaDesiredVelocity = new YoDouble("alphaDesiredVelocity", "Filter for velocity control in order to achieve acceleration control. Zero means compliant, but poor acceleration. One means stiff, but good acceleration tracking", registry);
   private final YoDouble kForceVel = new YoDouble("kForceVel", "Gain for velocity control in order to achieve acceleration control", registry);

   private final YoDouble qdd_tau = new YoDouble("qdd_tau", "Torque from inverse dynamics desired acceleration", registry);
   private final YoDouble qd_tau = new YoDouble("qd_tau", "Torque from integrating acceleration to get velocity", registry);
   private final YoDouble undelayedTorque = new YoDouble("undelayedTorque", "", registry);

   private final DelayedYoDouble delayedTorque = new DelayedYoDouble("delayedTorque", "", undelayedTorque, 2, registry);
   
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
