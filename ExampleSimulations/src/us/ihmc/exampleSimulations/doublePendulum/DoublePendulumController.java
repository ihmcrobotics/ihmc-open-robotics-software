package us.ihmc.exampleSimulations.doublePendulum;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotController.RobotController;

public class DoublePendulumController implements RobotController
{

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   private DoubleYoVariable tau_joint1, tau_joint2, q_joint1, q_joint2, qd_joint1, qd_joint2;
   private DoubleYoVariable k1, k2, k3, k4; // these are the controller gain parameters
   private final YoVariableRegistry registry = new YoVariableRegistry("DoublePendulumController");
   private String name;
   public DoublePendulumController(DoublePendulumRobot rob, String name)
   {
      this.name = name;

      // get variable references from the robot
      q_joint1 = (DoubleYoVariable)rob.getVariable("q_joint1");
      qd_joint1 = (DoubleYoVariable)rob.getVariable("qd_joint1");
      tau_joint1 = (DoubleYoVariable)rob.getVariable("tau_joint1");
      q_joint2 = (DoubleYoVariable)rob.getVariable("q_joint2");
      qd_joint2 = (DoubleYoVariable)rob.getVariable("qd_joint2");
      tau_joint2 = (DoubleYoVariable)rob.getVariable("tau_joint2");
      // set controller gains
      // gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
      k1 = new DoubleYoVariable("k1", registry);
      k1.set(-242.52);
      k2 = new DoubleYoVariable("k2", registry);
      k2.set(-96.33);
      k3 = new DoubleYoVariable("k3", registry);
      k3.set(-104.59);
      k4 = new DoubleYoVariable("k4", registry);
      k4.set(-49.05);
   }
   public void doControl()
   {
      tau_joint1.set(0.0); // free bearing
      // set the torque at the controlled second joint
      tau_joint2.set(-k1.getDoubleValue() * q_joint1.getDoubleValue() - k2.getDoubleValue() * q_joint2.getDoubleValue() - k3.getDoubleValue() * qd_joint1.getDoubleValue() - k4.getDoubleValue() * qd_joint2.getDoubleValue());
   }

   public YoVariableRegistry getYoVariableRegistry()
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