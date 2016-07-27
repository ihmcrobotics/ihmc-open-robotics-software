package us.ihmc.exampleSimulations.doublePendulum2;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class DoublePendulumController //implements RobotController
{

//   // tau_* is torque, q_* is position, qd_* is velocity for joint *
//   private DoubleYoVariable tau_joint1, tau_joint2, q_joint1, q_joint2, qd_joint1, qd_joint2;
//   public static final double L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;
//   private DoubleYoVariable k1, k2, k3, k4; // these are the controller gain parameters
//   private final YoVariableRegistry registry = new YoVariableRegistry("DoublePendulumController");
//   private String name;
//
//   private final DoubleYoVariable gravityCompensation;
//   private final DoubleYoVariable q_d;
//   private final DoubleYoVariable time;
//
//   public DoublePendulumController(DoublePendulumRobot rob, String name)
//   {
//      this.name = name;
//
//      // get variable references from the robot
//      q_joint1 = (DoubleYoVariable)rob.getVariable("q_joint1");
//      qd_joint1 = (DoubleYoVariable)rob.getVariable("qd_joint1");
//      tau_joint1 = (DoubleYoVariable)rob.getVariable("tau_joint1");
//      q_joint2 = (DoubleYoVariable)rob.getVariable("q_joint2");
//      qd_joint2 = (DoubleYoVariable)rob.getVariable("qd_joint2");
//      tau_joint2 = (DoubleYoVariable)rob.getVariable("tau_joint2");
//      // set controller gains
//      // gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
//      k1 = new DoubleYoVariable("k1", registry);
//      k1.set(100.0);
//      k2 = new DoubleYoVariable("k2", registry);
//      k2.set(100.0);
//      k3 = new DoubleYoVariable("k3", registry);
//      k3.set(-104.59);
//      k4 = new DoubleYoVariable("k4", registry);
//      k4.set(-49.05);
//
//      gravityCompensation = new DoubleYoVariable("gravityCompensation", registry);
//      q_d = new DoubleYoVariable("q_d", registry);
//
//      time = rob.getYoTime();
//   }
//   public void doControl()
//   {
//      q_d.set(Math.cos(0.1 * time.getDoubleValue()));
//
//      double q = q_joint1.getDoubleValue();
//
//      double proportionalTerm = k1.getDoubleValue() * (q_d.getDoubleValue() - q); // P = Proportional
//      double damping = - k2.getDoubleValue() * qd_joint1.getDoubleValue();        // D = Derivative / Damping
//      gravityCompensation.set( - DoublePendulumRobot.M1 * 9.81 * DoublePendulumRobot.L1 / 2.0 * Math.sin(q)); // alternative: I = Integral
//
//      tau_joint1.set(proportionalTerm + damping + gravityCompensation.getDoubleValue()); // free bearing
//      // set the torque at the controlled second joint
//    //  tau_joint2.set(0);
//
//      //tau_joint2.set(-k1.getDoubleValue() * q_joint1.getDoubleValue() - k2.getDoubleValue() * q_joint2.getDoubleValue() - k3.getDoubleValue() * qd_joint1.getDoubleValue() - k4.getDoubleValue() * qd_joint2.getDoubleValue());
//   }
//
//   public YoVariableRegistry getYoVariableRegistry()
//   {
//      return registry;
//   }
//
//   public String getName()
//   {
//      return name;
//   }
//
//   public void initialize()
//   {
//   }
//   public String getDescription()
//   {
//      return getName();
//   }
}