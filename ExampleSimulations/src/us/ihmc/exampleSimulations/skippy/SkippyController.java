package us.ihmc.exampleSimulations.skippy;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class SkippyController implements RobotController
{

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   private DoubleYoVariable q_foot, q_hip, qd_foot, qd_hip;
   private DoubleYoVariable k1, k2, k3, k4; // these are the controller gain parameters

   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   private String name;
   private SkippyRobot robot;

   public SkippyController(SkippyRobot robot, String name)
   {
      this.name = name;
      this.robot = robot;

      // get variable references from the robot
      q_foot = (DoubleYoVariable)robot.getVariable("q_foot");
      qd_foot = (DoubleYoVariable)robot.getVariable("qd_foot");

      q_hip = (DoubleYoVariable)robot.getVariable("q_hip");
      qd_hip = (DoubleYoVariable)robot.getVariable("qd_hip");

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
      // set the torques
      robot.getHipJoint().setTau(-k1.getDoubleValue() * q_foot.getDoubleValue() - k2.getDoubleValue() * q_hip.getDoubleValue() - k3.getDoubleValue() * qd_foot.getDoubleValue() - k4.getDoubleValue() * qd_hip.getDoubleValue());
      robot.getShoulderJoint().setTau(-k1.getDoubleValue() * q_foot.getDoubleValue() - k2.getDoubleValue() * q_hip.getDoubleValue() - k3.getDoubleValue() * qd_foot.getDoubleValue() - k4.getDoubleValue() * qd_hip.getDoubleValue());
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