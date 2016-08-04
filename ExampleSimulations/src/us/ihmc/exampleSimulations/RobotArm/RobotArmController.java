package us.ihmc.exampleSimulations.RobotArm;


import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class RobotArmController implements RobotController
{

       public static String variableName = "Turning1";
      public static String variableName2 = "longstick1";
      public static String variableName3 = "longstick2";
      public static String variableName4 = "Turning2";
      public static String variableName5 = "longstick3";
      public static String variableName6 = "Turning3";
      public static String variableName7 = "longstick4";

      // tau_* is torque, q_* is position, qd_* is velocity for joint *
      private DoubleYoVariable q_circle, q_stick1,q_circle2, q_stick2, q_stick3,q_circle3, q_stick4;
      private DoubleYoVariable k1, k2, k3, k4; // these are the controller gain parameters
      private final YoVariableRegistry registry = new YoVariableRegistry("DoublePendulumController");
   private final DoubleYoVariable circle1 = new DoubleYoVariable(variableName, registry);
   private final DoubleYoVariable stick1 = new DoubleYoVariable(variableName2, registry);

   private final DoubleYoVariable stick2 = new DoubleYoVariable(variableName3, registry);
   private final DoubleYoVariable circle2 = new DoubleYoVariable(variableName4, registry);
   private final DoubleYoVariable stick3 = new DoubleYoVariable(variableName5, registry);
   private final DoubleYoVariable circle3 = new DoubleYoVariable(variableName6, registry);
   private final DoubleYoVariable stick4 = new DoubleYoVariable(variableName7, registry);
      private String name;
      public RobotArmController(RobotArm rob, String name)
      {
         this.name = name;

         // get variable references from the robot
         q_circle = (DoubleYoVariable)rob.getVariable("q_circle");
         q_stick1= (DoubleYoVariable)rob.getVariable("q_stick1");
         q_circle2 = (DoubleYoVariable)rob.getVariable("q_circle2");
         q_stick2 = (DoubleYoVariable)rob.getVariable("q_stick2");
         q_stick3 = (DoubleYoVariable)rob.getVariable("q_stick3");
         q_circle3 = (DoubleYoVariable)rob.getVariable("q_circle3");
         q_stick4 = (DoubleYoVariable)rob.getVariable("q_stick4");

      }
      public void doControl()
      {
         q_circle.set(circle1.getDoubleValue());
       //  System.out.println(circle1.getDoubleValue());
         q_stick1.set(stick1.getDoubleValue());
         q_circle2.set(circle2.getDoubleValue());
         q_stick2.set(stick2.getDoubleValue());
         q_stick3.set(stick3.getDoubleValue());
         q_circle3.set(circle3.getDoubleValue());
         q_stick4.set(stick4.getDoubleValue());
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
