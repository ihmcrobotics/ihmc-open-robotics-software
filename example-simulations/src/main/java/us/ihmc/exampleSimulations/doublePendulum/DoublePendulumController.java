package us.ihmc.exampleSimulations.doublePendulum;

import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoublePendulumController implements RobotController
{

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   private YoDouble tau_joint1, tau_joint2, q_joint1, q_joint2, qd_joint1, qd_joint2;
   private YoDouble k1, k2, k3, k4; // these are the controller gain parameters
   private final YoRegistry registry = new YoRegistry("DoublePendulumController");
   private String name;
   public DoublePendulumController(DoublePendulumRobot rob, String name)
   {
      this.name = name;

      // get variable references from the robot
      q_joint1 = (YoDouble)rob.findVariable("q_joint1");
      qd_joint1 = (YoDouble)rob.findVariable("qd_joint1");
      tau_joint1 = (YoDouble)rob.findVariable("tau_joint1");
      q_joint2 = (YoDouble)rob.findVariable("q_joint2");
      qd_joint2 = (YoDouble)rob.findVariable("qd_joint2");
      tau_joint2 = (YoDouble)rob.findVariable("tau_joint2");
      // set controller gains
      // gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
      k1 = new YoDouble("k1", registry);
      k1.set(-242.52);
      k2 = new YoDouble("k2", registry);
      k2.set(-96.33);
      k3 = new YoDouble("k3", registry);
      k3.set(-104.59);
      k4 = new YoDouble("k4", registry);
      k4.set(-49.05);
   }
   public void doControl()
   {
      tau_joint1.set(0.0); // free bearing
      // set the torque at the controlled second joint
      tau_joint2.set(-k1.getDoubleValue() * q_joint1.getDoubleValue() - k2.getDoubleValue() * q_joint2.getDoubleValue() - k3.getDoubleValue() * qd_joint1.getDoubleValue() - k4.getDoubleValue() * qd_joint2.getDoubleValue());
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public void initialize()
   {
   }
   public String getDescription()
   {
      return getName();
   }
}