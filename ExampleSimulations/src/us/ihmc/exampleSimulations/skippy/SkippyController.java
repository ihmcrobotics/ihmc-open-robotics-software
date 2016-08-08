package us.ihmc.exampleSimulations.skippy;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.mathfunctions.Matrix;
import us.ihmc.simulationconstructionset.robotController.RobotController;

import javax.vecmath.*;
import java.util.ArrayList;
import java.util.Vector;

public class SkippyController implements RobotController
{

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
//   private DoubleYoVariable q_foot_X, q_hip, q_shoulder, qd_foot_X, qd_hip, qd_shoulder;
   private DoubleYoVariable k1, k2, k3, k4, k5, k6, k7, k8, an, vel; // controller gain parameters

   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   private String name;
   private SkippyRobot robot;

   private static ArrayList<double[]> desiredPositions;

   private double legIntegralTermX = 0.0;
   private double legIntegralTermY = 0.0;
   private double hipIntegralTerm = 0.0;
   private double shoulderIntegralTerm = 0.0;

   private static int timeCounter = 0;
   private static double prevAngleHip = -Math.PI+Math.PI/24;

   public SkippyController(SkippyRobot robot, String name)
   {
      this.name = name;
      this.robot = robot;

      // get variable references from the robot
//      q_foot_X = (DoubleYoVariable)robot.getVariable("q_foot_X");
//      qd_foot_X = (DoubleYoVariable)robot.getVariable("qd_foot_X");
//
//      q_hip = (DoubleYoVariable)robot.getVariable("q_hip");
//      qd_hip = (DoubleYoVariable)robot.getVariable("qd_hip");
//
//      q_shoulder = (DoubleYoVariable)robot.getVariable("q_shoulder");
//      qd_shoulder = (DoubleYoVariable)robot.getVariable("qd_shoulder");

      // set controller gains
      /* gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
         k1 = -242.52
         k2 = -96.33
         k3 = -104.59
         k4 = -49.05
       */
      k1 = new DoubleYoVariable("k1", registry);
      k1.set(43);
      k2 = new DoubleYoVariable("k2", registry);
      k2.set(-30);
      k3 = new DoubleYoVariable("k3", registry);
      k3.set(20);
      k4 = new DoubleYoVariable("k4", registry);
      k4.set(-10);

      k5 = new DoubleYoVariable("k5", registry);
      k5.set(59);
      k6 = new DoubleYoVariable("k6", registry);
      k6.set(26.33);
      k7 = new DoubleYoVariable("k7", registry);
      k7.set(22.59);
      k8 = new DoubleYoVariable("k8", registry);
      k8.set(4.05);

      an = new DoubleYoVariable("an", registry);
      vel = new DoubleYoVariable("vel", registry);
      
      //for show
      desiredPositions = new ArrayList<double[]>();
      //double[] position = {qLegDesiredX, qLegDesiredY, qHipDesired, qShoulderDesired} <- format
      double[] firstSetOfPoints = {0.0,0.0,0.0,0.0};
      double[] secondSetOfPoints = {Math.PI/4, 0.0, -2*Math.PI/4, 0.0};
      double[] thirdSetOfPoints = {-Math.PI/4, 0.0, 2*Math.PI/4, 0.0};
      //desiredPositions.add(firstSetOfPoints);
      desiredPositions.add(secondSetOfPoints);
      desiredPositions.add(thirdSetOfPoints);
      desiredPositions.add(firstSetOfPoints);
   }

   public void doControl()
   {
      // set the torques

      //start pid control
      //System.out.println(this.robot.mainJoint.getQdy());
      //positionControl();

      balanceControl(0.0, 0.0);
   }

   private void balanceControl(double hipDesired, double shoulderDesired)
   {
      applyTorqueToHip(hipDesired);
      //applyTorqueToShoulder(shoulderDesired);
   }

   private void applyTorqueToHip(double hipDesired)
   {
      Point3d centerOfMass = new Point3d();
      double robotMass = robot.computeCenterOfMass(centerOfMass);
      Point3d groundPoint = new Point3d();
      robot.getGroundContactPoints().get(0).getPosition(groundPoint);
      double planarDistance = Math.pow(Math.pow(centerOfMass.getY()-groundPoint.getY(), 2) + Math.pow(centerOfMass.getZ()-groundPoint.getZ(), 2), 0.5);
      double angle = (Math.asin(Math.abs(centerOfMass.getZ())/planarDistance));

      if(centerOfMass.getZ()>0)
         angle = Math.PI/2 - angle;
      else
         angle = Math.PI/2 + angle;

      if(centerOfMass.getY()<0)
         angle = angle * -1;

      an.set(fromRadiansToDegrees(angle));

//      System.out.println(centerOfMass);
//      System.out.println(angle);
//      System.out.println(angle*180/Math.PI);
//      if(true)
//         return;

//      Vector3d linearMomentum = new Vector3d();
//      robot.computeLinearMomentum(robot.getHipJoint(), linearMomentum);
//      double angleVel = Math.pow(Math.pow(linearMomentum.getY(), 2) + Math.pow(linearMomentum.getZ(), 2), 0.5)/robotMass;
//      //double angleVel = linearMomentum.length()/robotMass;
//      angleVel = angleVel / planarDistance;

      //if(angle < prevAngleHip)
      //   angleVel = angleVel * -1;
      double angleVel = (angle - prevAngleHip) / SkippySimulation.DT;
      prevAngleHip = angle;

      vel.set(angleVel);

      double[] hipAngleValues = calculateAnglePosAndDerOfJoint(robot.getHipJoint());
      double hipAngle = hipAngleValues[0];
      double hipAngleVel = hipAngleValues[1];
      robot.getHipJoint().setTau(k1.getDoubleValue()*(angle) + k2.getDoubleValue()*angleVel + k3.getDoubleValue()*(hipDesired-hipAngle) + k4.getDoubleValue()*hipAngleVel);
      //robot.getHipJoint().setTau(k1.getDoubleValue()*angle + k2.getDoubleValue()*angleVel);
      //robot.getHipJoint().setTau(k3.getDoubleValue()*(hipDesired-hipAngle) + k4.getDoubleValue()*hipAngleVel);
      System.out.println(centerOfMass + " " + fromRadiansToDegrees(angle) + " " + angleVel + " " + (hipDesired-hipAngle) + " " + hipAngleVel + " " + robot.getHipJoint().getTau());
   }
   private void applyTorqueToShoulder(double shoulderDesired)
   {
      Point3d centerOfMass = new Point3d();
      double robotMass = robot.computeCenterOfMass(robot.getLegJoint().getSecondJoint(), centerOfMass);
      Point3d groundPoint = new Point3d();
      robot.getGroundContactPoints().get(0).getPosition(groundPoint);
      double planarDistance = Math.pow(Math.pow(centerOfMass.getX()-groundPoint.getX(), 2) + Math.pow(centerOfMass.getZ()-groundPoint.getZ(), 2), 0.5);
      double angle = (Math.asin(centerOfMass.getX()/planarDistance));
      if(centerOfMass.getZ()<0)
         angle = angle+Math.PI/2;

      Vector3d linearMomentum = new Vector3d();
      robot.computeLinearMomentum(linearMomentum);
      double angleVel = Math.pow(Math.pow(linearMomentum.getX(), 2) + Math.pow(linearMomentum.getZ(), 2), 0.5)/robotMass;
      angleVel = angleVel / planarDistance;
      if(centerOfMass.getZ()<0)
         angleVel *= -1;
      vel.set(angleVel);

      double[] hipAngleValues = calculateAnglePosAndDerOfJoint(robot.getShoulderJoint());
      double hipAngle = hipAngleValues[0];
      double hipAngleVel = hipAngleValues[1];

      robot.getShoulderJoint().setTau(k1.getDoubleValue()*angle + k2.getDoubleValue()*angleVel + k3.getDoubleValue()*(shoulderDesired-hipAngle) + k4.getDoubleValue()*hipAngleVel);

      System.out.println(fromRadiansToDegrees(angle) + " " + angleVel + " " + (shoulderDesired-hipAngle) + " " + hipAngleVel);
   }
   private double[] calculateAnglePosAndDerOfJoint(PinJoint joint)
   {
      double[] finale = new double[2];
      double angle = joint.getQ().getDoubleValue();
//      if(angle < 0)
//         angle = Math.PI - angle*-1;
//      else
//         angle = Math.PI - angle;
      double angleVel = joint.getQD().getDoubleValue();
      //angle = (angle * 180/Math.PI)%360;
      finale[0] = angle*Math.PI/180;
      finale[1] = (angleVel);
      return finale;
   }
   private double calculateDistanceBetweenPoints(Point3d a, Point3d b)
   {
      return a.distance(b);
   }
   private double fromRadiansToDegrees(double radians)
   {
      return radians * 180 / Math.PI;
   }


   private void positionControl()
   {
      double interval = Math.round(SkippySimulation.TIME/desiredPositions.size()*100.0)/100.0;
      double time = Math.round(robot.getTime()*(1/SkippySimulation.DT))/(1/SkippySimulation.DT);

      positionJointsBasedOnError(robot.getLegJoint(),desiredPositions.get(timeCounter)[0], legIntegralTermX, 20000, 150, 2000, true);
      positionJointsBasedOnError(robot.getLegJoint().getSecondJoint(), desiredPositions.get(timeCounter)[1], legIntegralTermY, 20000, 150, 2000, false);
      positionJointsBasedOnError(robot.getHipJoint(), desiredPositions.get(timeCounter)[2], hipIntegralTerm, 20000, 150, 2000, false);
      positionJointsBasedOnError(robot.getShoulderJoint(), desiredPositions.get(timeCounter)[3], shoulderIntegralTerm, 20000, 150, 2000, false);
      //System.out.println();

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

   public void positionJointsBasedOnError(PinJoint joint, double desiredValue, double integralTerm, double positionErrorGain, double integralErrorGain, double derivativeErrorGain, boolean isBasedOnWorldCoordinates)
   {
      //try to change position based on angular position wrt xyz coordinate system
      Matrix3d rotationMatrixForWorld = new Matrix3d();
      joint.getRotationToWorld(rotationMatrixForWorld);
      double rotationToWorld = Math.asin((rotationMatrixForWorld.getM21()));
      //if(rotationMatrixForWorld.getM11()<0)
      //   rotationToWorld = rotationToWorld * -1;
      if(isBasedOnWorldCoordinates)
         System.out.println(joint.getName() + " " + (joint.getQ().getDoubleValue()) + " " + rotationToWorld);
      else
         rotationToWorld = joint.getQ().getDoubleValue();


      double positionError = (positionErrorGain)*((desiredValue-rotationToWorld));
      integralTerm += (integralErrorGain)*positionError*SkippySimulation.DT;
      double derivativeError = (derivativeErrorGain)*(0-joint.getQD().getDoubleValue());
      joint.setTau(positionError+integralTerm+derivativeError);
      //System.out.print(joint.getName() + ": " + (joint.getQ().getDoubleValue() - desiredValue));
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
