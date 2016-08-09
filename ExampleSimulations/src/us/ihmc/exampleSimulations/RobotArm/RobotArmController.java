package us.ihmc.exampleSimulations.RobotArm;


import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class RobotArmController implements RobotController
{

   public static String variableName1 = "axis1";
   public static String variableName2 = "axis2";
   public static String variableName3 = "axis3";
   public static String variableName4 = "axis4";
   public static String variableName5 = "axis5";
   public static String variableName6 = "axis6";
   public static String variableName7 = "axis7Left";
   public static String variableName8 = "axis7Right";

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   private DoubleYoVariable q_axis1, q_axis2, q_axis4, q_axis3, q_axis5, q_axis6, q_axis7Left, q_axis7Right;
   private DoubleYoVariable k1, k2, k3, k4; // these are the controller gain parameters
   private final YoVariableRegistry registry = new YoVariableRegistry("RobotArmController");

   private final DoubleYoVariable axis1 = new DoubleYoVariable(variableName1, registry);
   private final DoubleYoVariable axis2 = new DoubleYoVariable(variableName2, registry);
   private final DoubleYoVariable axis3 = new DoubleYoVariable(variableName3, registry);
   private final DoubleYoVariable axis4 = new DoubleYoVariable(variableName4, registry);
   private final DoubleYoVariable axis5 = new DoubleYoVariable(variableName5, registry);
   private final DoubleYoVariable axis6 = new DoubleYoVariable(variableName6, registry);
   private final DoubleYoVariable axis7Left = new DoubleYoVariable(variableName7, registry);
   private final DoubleYoVariable axis7Right = new DoubleYoVariable(variableName8, registry);
   private String robotName;

   public RobotArmController(RobotArm robot, String robotName)
   {
      this.robotName = robotName;

      // get variable references from the robot
      q_axis1 = (DoubleYoVariable) robot.getVariable("q_axis1");
      q_axis2 = (DoubleYoVariable) robot.getVariable("q_axis2");
      q_axis3 = (DoubleYoVariable) robot.getVariable("q_axis3");
      q_axis4 = (DoubleYoVariable) robot.getVariable("q_axis4");
      q_axis5 = (DoubleYoVariable) robot.getVariable("q_axis5");
      q_axis6 = (DoubleYoVariable) robot.getVariable("q_axis6");
      q_axis7Left = (DoubleYoVariable) robot.getVariable("q_axis7Left");
      q_axis7Right = (DoubleYoVariable) robot.getVariable("q_axis7Right");
   }

   public void doControl()
   {
      q_axis1.set(axis1.getDoubleValue());
      q_axis2.set(axis2.getDoubleValue());
      q_axis3.set(axis3.getDoubleValue());
      q_axis4.set(axis4.getDoubleValue());
      q_axis5.set(axis5.getDoubleValue());
      q_axis6.set(axis6.getDoubleValue());
      q_axis7Left.set(axis7Left.getDoubleValue());
      q_axis7Right.set(-axis7Left.getDoubleValue());

      // tau_joint1.set(0.0); // free bearing
      // set the torque at the controlled second joint
      // tau_joint2.set(-k1.getDoubleValue() * q_joint1.getDoubleValue() - k2.getDoubleValue() * q_joint2.getDoubleValue() - k3.getDoubleValue() * qd_joint1.getDoubleValue() - k4.getDoubleValue() * qd_joint2.getDoubleValue());
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return robotName;
   }

   public void initialize()
   {
   }

   public String getDescription()
   {
      return getName();
   }
}
