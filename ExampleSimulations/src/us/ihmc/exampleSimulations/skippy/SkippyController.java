package us.ihmc.exampleSimulations.skippy;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class SkippyController implements RobotController
{

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   private DoubleYoVariable q_foot_X, q_hip, q_shoulder, qd_foot_X, qd_hip, qd_shoulder;
   private DoubleYoVariable k1, k2, k3, k4, k5, k6, k7, k8; // controller gain parameters

   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   private String name;
   private SkippyRobot robot;

   private double integralTerm = 0.0;
   private double integralTermPart2 = 0.0;
   private double integralTerm2 = 0.0;
   private double integralTerm3 = 0.0;

   public SkippyController(SkippyRobot robot, String name)
   {
      this.name = name;
      this.robot = robot;

      // get variable references from the robot
      q_foot_X = (DoubleYoVariable)robot.getVariable("q_foot_X");
      qd_foot_X = (DoubleYoVariable)robot.getVariable("qd_foot_X");

      q_hip = (DoubleYoVariable)robot.getVariable("q_hip");
      qd_hip = (DoubleYoVariable)robot.getVariable("qd_hip");

      q_shoulder = (DoubleYoVariable)robot.getVariable("q_shoulder");
      qd_shoulder = (DoubleYoVariable)robot.getVariable("qd_shoulder");

      // set controller gains
      /* gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
         k1 = -242.52
         k2 = -96.33
         k3 = -104.59
         k4 = -49.05
       */
      k1 = new DoubleYoVariable("k1", registry);
      k1.set(-242.52);
      k2 = new DoubleYoVariable("k2", registry);
      k2.set(-96.33);
      k3 = new DoubleYoVariable("k3", registry);
      k3.set(-104.59);
      k4 = new DoubleYoVariable("k4", registry);
      k4.set(-49.05);

      k5 = new DoubleYoVariable("k5", registry);
      k5.set(-242.52);
      k6 = new DoubleYoVariable("k6", registry);
      k6.set(-96.33);
      k7 = new DoubleYoVariable("k7", registry);
      k7.set(-104.59);
      k8 = new DoubleYoVariable("k8", registry);
      k8.set(-49.05);
   }

   public void doControl()
   {
      // desired hip/shoulder positions
      double q_leg_desired = -0.2;
      
      double q_leg_desired_2 = 0.0;

      double q_hip_desired = 0.4;
      double q_shoulder_desired = 1.8;
      // set the torques

//      robot.getHipJoint().setTau(-k1.getDoubleValue() * q_foot_X.getDoubleValue()
//                                       - k2.getDoubleValue() * (q_hip.getDoubleValue() - q_hip_desired)
//                                       - k3.getDoubleValue() * qd_foot_X.getDoubleValue()
//                                       - k4.getDoubleValue() * qd_hip.getDoubleValue());
//      robot.getShoulderJoint().setTau(-k5.getDoubleValue() * q_hip.getDoubleValue()
//                                            - k6.getDoubleValue() * (q_shoulder.getDoubleValue() - q_shoulder_desired)
//                                            - k7.getDoubleValue() * qd_hip.getDoubleValue()
//                                            - k8.getDoubleValue() * qd_shoulder.getDoubleValue());
      double positionError = (5000)*((q_leg_desired-robot.getLegJoint().getQ().getDoubleValue()));
      integralTerm += (1)*positionError*SkippySimulation.DT;
      double velocityError = (1000)*(0-robot.getLegJoint().getQD().getDoubleValue());
      robot.getLegJoint().setTau(positionError+integralTerm+velocityError);

      positionError = (5000)*((q_leg_desired_2-robot.getLegJoint().getSecondJoint().getQ().getDoubleValue()));
      integralTermPart2 += (1)*positionError*SkippySimulation.DT;
      velocityError = (1000)*(0-robot.getLegJoint().getSecondJoint().getQD().getDoubleValue());
      robot.getLegJoint().getSecondJoint().setTau(positionError+integralTerm+velocityError);
//
      positionError = (5000)*(q_hip_desired-robot.getHipJoint().getQ().getDoubleValue());
      integralTerm3 += (1)*positionError*SkippySimulation.DT;
      velocityError = (1000)*(0-robot.getHipJoint().getQD().getDoubleValue());
      robot.getHipJoint().setTau(positionError+integralTerm3+velocityError);
//
      positionError = (1000)*(q_shoulder_desired-robot.getShoulderJoint().getQ().getDoubleValue());
      integralTerm2 += (1)*positionError*SkippySimulation.DT;
      velocityError = (0.1)*(0-robot.getShoulderJoint().getQD().getDoubleValue());
      robot.getShoulderJoint().setTau(positionError+integralTerm2+velocityError);
//
      System.out.println(robot.getLegJoint().getQ() + " " + robot.getHipJoint().getQ() + " " + robot.getShoulderJoint().getQ());

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