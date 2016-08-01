package us.ihmc.exampleSimulations.skippy;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

import java.util.ArrayList;

public class SkippyController implements RobotController
{

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
   private DoubleYoVariable q_foot_X, q_hip, q_shoulder, qd_foot_X, qd_hip, qd_shoulder;
   private DoubleYoVariable k1, k2, k3, k4, k5, k6, k7, k8; // controller gain parameters

   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   private String name;
   private SkippyRobot robot;

   private static ArrayList<double[]> desiredPositions;

   private double legIntegralTermX = 0.0;
   private double legIntegralTermY = 0.0;
   private double hipIntegralTerm = 0.0;
   private double shoulderIntegralTerm = 0.0;

   private static int timeCounter = 0;

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

      desiredPositions = new ArrayList<double[]>();
      //double[] position = {qLegDesiredX, qLegDesiredY, qHipDesired, qShoulderDesired} <- format
      double[] firstSetOfPoints = {0.0,0.0,0.0,0.0};
      double[] secondSetOfPoints = {Math.PI/4, 0.0, -2*Math.PI/4, 0.15};
      double[] thirdSetOfPoints = {-Math.PI/4, 0.0, 2*Math.PI/4, 0.0};
      desiredPositions.add(firstSetOfPoints);
      desiredPositions.add(secondSetOfPoints);
      desiredPositions.add(thirdSetOfPoints);
   }

   public void doControl()
   {
      // set the torques

//      robot.getHipJoint().setTau(-k1.getDoubleValue() * q_foot_X.getDoubleValue()
//                                       - k2.getDoubleValue() * (q_hip.getDoubleValue() - q_hip_desired)
//                                       - k3.getDoubleValue() * qd_foot_X.getDoubleValue()
//                                       - k4.getDoubleValue() * qd_hip.getDoubleValue());
//      robot.getShoulderJoint().setTau(-k5.getDoubleValue() * q_hip.getDoubleValue()
//                                            - k6.getDoubleValue() * (q_shoulder.getDoubleValue() - q_shoulder_desired)
//                                            - k7.getDoubleValue() * qd_hip.getDoubleValue()
//                                            - k8.getDoubleValue() * qd_shoulder.getDoubleValue());

      //start pid control
      //System.out.println(this.robot.mainJoint.getQdy());
      positionControl();
      //positionControl(0.0,0.0,0.0,0.0);
      //balanceControl(Math.PI/6, 0.0);

   }

   private void balanceControl(double thetaX, double thetaY)
   {

      double forceX, forceY;

      double positionError = (10)*(thetaX-robot.getLegJoint().getQ().getDoubleValue());
      legIntegralTermX += (1)*(positionError*SkippySimulation.DT);
      double derivativeError = (10)*(0.0-robot.getLegJoint().getQD().getDoubleValue());
      forceX = positionError+legIntegralTermX+derivativeError;

      positionError = (10)*(thetaY-robot.getLegJoint().getSecondJoint().getQ().getDoubleValue());
      legIntegralTermY += (1)*(positionError*SkippySimulation.DT);
      derivativeError = (10)*(0.0-robot.getLegJoint().getSecondJoint().getQD().getDoubleValue());
      forceY = positionError+legIntegralTermY+derivativeError;

      //forceY = 0.0;
      //forceX = 0.0;

      robot.setBalanceForce(forceX,0.0,forceY);

   }

   private void positionControl()
   {
      double interval = Math.round(SkippySimulation.TIME/desiredPositions.size()*100.0)/100.0;
      double time = Math.round(robot.getTime()*(1/SkippySimulation.DT))/(1/SkippySimulation.DT);

      positionJointsBasedOnError(robot.getLegJoint(),desiredPositions.get(timeCounter)[0], legIntegralTermX, 7000, 150, 1000);
      positionJointsBasedOnError(robot.getLegJoint().getSecondJoint(), desiredPositions.get(timeCounter)[1], legIntegralTermY, 7000, 150, 1000);
      positionJointsBasedOnError(robot.getHipJoint(), desiredPositions.get(timeCounter)[2], hipIntegralTerm, 7000, 150, 1000);
      positionJointsBasedOnError(robot.getShoulderJoint(), desiredPositions.get(timeCounter)[3], shoulderIntegralTerm, 7000, 150, 1000);
      System.out.println();

      if(time%interval==0 && time != 0.0 && timeCounter < desiredPositions.size())
      {
         timeCounter++;
         legIntegralTermX = 0;
         legIntegralTermY = 0;
         hipIntegralTerm = 0;
         shoulderIntegralTerm = 0;
      }
      if(timeCounter==desiredPositions.size())
         timeCounter = desiredPositions.size()-1;
   }

   public void positionJointsBasedOnError(PinJoint joint, double desiredValue, double integralTerm, double positionErrorGain, double integralErrorGain, double derivativeErrorGain)
   {
      double positionError = (positionErrorGain)*((desiredValue-+joint.getQ().getDoubleValue()));
      integralTerm += (integralErrorGain)*positionError*SkippySimulation.DT;
      double derivativeError = (derivativeErrorGain)*(0-joint.getQD().getDoubleValue());
      joint.setTau(positionError+integralTerm+derivativeError);
      System.out.print(joint.getName() + ": " + (joint.getQ().getDoubleValue() - desiredValue) + " " + joint.getTau() + " ");
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