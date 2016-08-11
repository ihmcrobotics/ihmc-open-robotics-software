package us.ihmc.exampleSimulations.skippy;

import java.util.ArrayList;
import java.util.Vector;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;

public class SkippyController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SkippyController");

   // tau_* is torque, q_* is position, qd_* is velocity for joint *
//   private DoubleYoVariable q_foot_X, q_hip, qHipIncludingOffset, qd_foot_X, qd_hip, qd_shoulder;
   private final DoubleYoVariable k1, k2, k3, k4, k5, k6, k7, k8, angleToCoMInYZPlane, angleToCoMInXZPlane, angularVelocityToCoMYZPlane, angularVelocityToCoMXZPlane; // controller gain parameters
   private final DoubleYoVariable planarDistanceYZPlane, planarDistanceXZPlane;

   private final YoFramePoint centerOfMass = new YoFramePoint("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint footLocation = new YoFramePoint("foot", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleYoVariable robotMass = new DoubleYoVariable("robotMass", registry);
   private final DoubleYoVariable qHipIncludingOffset = new DoubleYoVariable("qHipIncludingOffset", registry);
   private final DoubleYoVariable q_d_hip = new DoubleYoVariable("q_d_hip", registry);
   private final DoubleYoVariable qShoulderIncludingOffset = new DoubleYoVariable("qShoulderIncludingOffset", registry);
   private final DoubleYoVariable q_d_shoulder = new DoubleYoVariable("q_d_shoulder", registry);

   private String name;
   private SkippyRobot robot;
   private int robotType;

   private static ArrayList<double[]> desiredPositions;

   private double legIntegralTermX = 0.0;
   private double legIntegralTermY = 0.0;
   private double hipIntegralTerm = 0.0;
   private double shoulderIntegralTerm = 0.0;

   private static int timeCounter = 0;
   private static double prevAngleHip = -Math.PI+Math.PI/6;

   public SkippyController(SkippyRobot robot, int robotType, String name)
   {
      this.name = name;
      this.robot = robot;
      this.robotType = robotType;

      // get variable references from the robot
//      q_foot_X = (DoubleYoVariable)robot.getVariable("q_foot_X");
//      qd_foot_X = (DoubleYoVariable)robot.getVariable("qd_foot_X");
//
//      q_hip = (DoubleYoVariable)robot.getVariable("q_hip");
//      qd_hip = (DoubleYoVariable)robot.getVariable("qd_hip");
//
//      qHipIncludingOffset = (DoubleYoVariable)robot.getVariable("qHipIncludingOffset");
//      qd_shoulder = (DoubleYoVariable)robot.getVariable("qd_shoulder");

      // set controller gains
      /* gains taken from Mark Spong (1995) "The Swing Up Control Problem for the Acrobot"
         k1 = -242.52
         k2 = -96.33
         k3 = -104.59
         k4 = -49.05
       */
      k1 = new DoubleYoVariable("k1", registry);
      k2 = new DoubleYoVariable("k2", registry);
      k3 = new DoubleYoVariable("k3", registry);
      k4 = new DoubleYoVariable("k4", registry);
      k5 = new DoubleYoVariable("k5", registry);
      k6 = new DoubleYoVariable("k6", registry);
      k7 = new DoubleYoVariable("k7", registry);
      k8 = new DoubleYoVariable("k8", registry);

      if(robotType == 0)
      {
         k1.set(3600.0); //110);
         k2.set(1100.0); //-35);
         k3.set(-170.0); //30);
         k4.set(-130.0); //-15);

         k5.set(-1900);
         k6.set(-490.0);
         k7.set(-60.0);
         k8.set(-45.0);
      }
      else if(robotType == 1)
      {
         k1.set(2500.0); //110);
         k2.set(0.0); //-35);
         k3.set(-25.0); //30);
         k4.set(-0.0); //-15);

         k5.set(-2500);
         k6.set(-0.0);
         k7.set(-0.0);
         k8.set(-0.0);
      }

      q_d_hip.set(-0.6);
      q_d_shoulder.set(0.0);

      planarDistanceYZPlane = new DoubleYoVariable("planarDistanceYZPlane", registry);
      planarDistanceXZPlane = new DoubleYoVariable("planarDistanceXZPlane", registry);
      angleToCoMInYZPlane = new DoubleYoVariable("angleToCoMYZPlane", registry);
      angleToCoMInXZPlane = new DoubleYoVariable("angleToCoMXZPlane", registry);
      angularVelocityToCoMYZPlane = new DoubleYoVariable("angularVelocityToCoMYZPlane", registry);
      angularVelocityToCoMXZPlane = new DoubleYoVariable("angularVelocityToCoMXZPlane", registry);

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
      computeCenterOfMass();
      computeFootLocation();
      balanceControl(q_d_hip.getDoubleValue(), q_d_shoulder.getDoubleValue());

      // set the torques

      //start pid control
      //System.out.println(this.robot.mainJoint.getQdy());
      //positionControl();
   }

   private void balanceControl(double hipDesired, double shoulderDesired)
   {
      applyTorqueToHip(hipDesired);
      applyTorqueToShoulder(shoulderDesired);
   }

   private void computeFootLocation()
   {
      Point3d groundPoint = new Point3d();
      robot.getGroundContactPoints().get(0).getPosition(groundPoint);
      footLocation.set(groundPoint);
   }

   private void computeCenterOfMass()
   {
      Point3d tempCenterOfMass = new Point3d();
      robotMass.set(robot.computeCenterOfMass(tempCenterOfMass));
      centerOfMass.set(tempCenterOfMass);
   }

   private void applyTorqueToHip(double hipDesired)
   {
      /*
         angular pos : angle created w/ com to groundpoint against vertical
       */

      double footToComZ = centerOfMass.getZ()-footLocation.getZ();
      double footToComY = centerOfMass.getY()-footLocation.getY();

      planarDistanceYZPlane.set(Math.sqrt(Math.pow(centerOfMass.getY()-footLocation.getY(), 2) + Math.pow(footToComZ, 2)));
      double angle = (Math.atan2(footToComY, footToComZ));
      angleToCoMInYZPlane.set(fromRadiansToDegrees(angle));

      /*
         angular vel : angle created w/ com to groundpoint against vertical
       */
      Vector3d linearMomentum = new Vector3d();
      robot.computeLinearMomentum(linearMomentum);

      //1: projection vector
      Vector3d componentPerpendicular = new Vector3d(0, 1, -centerOfMass.getY()/centerOfMass.getZ());
      componentPerpendicular.normalize();
      double angleVel = componentPerpendicular.dot(linearMomentum) / componentPerpendicular.length();
      angleVel = angleVel / robotMass.getDoubleValue();

      //2: not used
      //double angleVel = Math.pow(Math.pow(linearMomentum.getY(), 2) + Math.pow(linearMomentum.getZ(), 2), 0.5)/robotMass;
      //angleVel = angleVel / planarDistanceYZPlane;

      //3: average rate of change (buggy)
      //double angleVel = (angle - prevAngleHip) / SkippySimulation.DT;

      angularVelocityToCoMYZPlane.set(angleVel);


      /*
         angular pos/vel of hipjoint
       */
      double hipAngle = 0;
      double hipAngleVel = 0;
      if(robotType == 0)
      {
         double[] hipAngleValues = calculateAnglePosAndDerOfHipJointAcrobot(robot.getHipJointAcrobot());
         hipAngle = hipAngleValues[0];
         hipAngleVel = hipAngleValues[1];
         qHipIncludingOffset.set(hipAngle);
      }
      else if(robotType == 1)
      {
         double[] hipAngleValues = calculateAnglePosAndDerOfHipJointSkippy(robot.getHipJointSkippy());
         hipAngle = hipAngleValues[0];
         hipAngleVel = hipAngleValues[1];
         //qHipIncludingOffset.set(fromRadiansToDegrees(hipAngle));
      }


      //torque set
      if(robotType == 0)
      {
         robot.getHipJointAcrobot().setTau(k1.getDoubleValue() * (0.0 - angle) + k2.getDoubleValue() * (0.0 - angleVel) + k3.getDoubleValue() * (hipDesired - hipAngle) + k4.getDoubleValue() * (0.0 - hipAngleVel));
      }
      else if(robotType == 1)
      {
         //torque ~> force

         double tau = k1.getDoubleValue() * (0.0 - angle) + k2.getDoubleValue() * (0.0 - angleVel) + k3.getDoubleValue() * (hipDesired - hipAngle) + k4.getDoubleValue() * (0.0 - hipAngleVel);
         Vector3d point2 = createVectorInDirectionOfHipJoint();
         Vector3d forceDirectionVector = new Vector3d(0, 1, point2.getY()/point2.getZ()*-1);
         forceDirectionVector.normalize();
         forceDirectionVector.scale(tau/robot.getHipLength()/2);
         robot.setRootJointForce(forceDirectionVector.getX(), forceDirectionVector.getY(), forceDirectionVector.getZ());
      }
   }
   private Vector3d createVectorInDirectionOfHipJoint()
   {
      Vector3d point1 = new Vector3d();
      robot.getHipJointSkippy().getTranslationToWorld(point1);
      Vector3d point2 = new Vector3d();
      robot.getGroundContactPoints().get(1).getPosition(point2);
      point1.sub(point2);
      return point1;
   }

   private void applyTorqueToShoulder(double shoulderDesired)
   {
      /*
         angular pos : angle created w/ com to groundpoint against vertical
       */

      double footToComZ = centerOfMass.getZ()-footLocation.getZ();
      double footToComX = centerOfMass.getX()-footLocation.getX();

      planarDistanceXZPlane.set(Math.sqrt(Math.pow(footToComX, 2) + Math.pow(footToComZ, 2)));
      double angle = (Math.atan2(footToComX, footToComZ));
      angleToCoMInXZPlane.set(fromRadiansToDegrees(angle));

      /*
         angular vel : angle created w/ com to groundpoint against vertical
       */
      Vector3d linearMomentum = new Vector3d();
      robot.computeLinearMomentum(linearMomentum);

      //1: projection vector
      Vector3d componentPerpendicular = new Vector3d(1, 0, -centerOfMass.getX()/centerOfMass.getZ());
      componentPerpendicular.normalize();
      double angleVel = componentPerpendicular.dot(linearMomentum) / componentPerpendicular.length();
      angleVel = angleVel / robotMass.getDoubleValue();

      //2: not used
      //double angleVel = Math.pow(Math.pow(linearMomentum.getY(), 2) + Math.pow(linearMomentum.getZ(), 2), 0.5)/robotMass;
      //angleVel = angleVel / planarDistanceYZPlane;

      //3: average rate of change (buggy)
      //double angleVel = (angle - prevAngleHip) / SkippySimulation.DT;

      angularVelocityToCoMXZPlane.set(angleVel);


      /*
         angular pos/vel of hipjoint
       */
      double[] shoulderAngleValues = calculateAnglePosAndDerOfShoulderJointAcrobot(robot.getShoulderJoint());
      double shoulderAngle = shoulderAngleValues[0];
      double shoulderAngleVel = shoulderAngleValues[1];
      qShoulderIncludingOffset.set((shoulderAngle));

      robot.getShoulderJoint().setTau(k5.getDoubleValue()*Math.sin(0.0-angle) + k6.getDoubleValue()*(0.0 - angleVel) + k7.getDoubleValue()*(shoulderDesired-shoulderAngle) + k8.getDoubleValue()*(0.0 - shoulderAngleVel));

   }

   private double[] calculateAnglePosAndDerOfHipJointAcrobot(PinJoint joint)
   {
      double[] finale = new double[2];
      double firstAngle = robot.getLegJoint().getQ().getDoubleValue()%(Math.PI*2);
      if(firstAngle>Math.PI)
         firstAngle = (Math.PI*2-firstAngle)*-1;
      double angle = (joint.getQ().getDoubleValue())%(Math.PI*2)+firstAngle;
      if(angle > Math.PI)
         angle = angle - Math.PI*2;

      //System.out.println(fromRadiansToDegrees(firstAngle) + " " + fromRadiansToDegrees(angle));

      //double angle = joint.getQ().getDoubleValue();

      double angleVel = joint.getQD().getDoubleValue();
      //angle = (angle * 180/Math.PI)%360;
      finale[0] = angle;
      finale[1] = (angleVel);
      return finale;
   }

   private double[] calculateAnglePosAndDerOfHipJointSkippy(FloatingJoint joint)  //doesnt work if hipContactPoint comes in contact with ground due to offset issues
   {
      double[] finale = new double[2];
      double firstAngle = (robot.getLegJoint().getQ().getDoubleValue()-Math.PI)%(Math.PI*2);
      if(firstAngle>Math.PI)
         firstAngle = (Math.PI*2-firstAngle)*-1;

      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);
      Vector3d floatVector = createVectorInDirectionOfHipJoint();

      double cosineTheta = (floatVector.dot(verticalVector)/(floatVector.length() * verticalVector.length()));

      double angle = (-1)*Math.acos(cosineTheta);
      double angleVel = (-1)*robot.getLegJoint().getQD().getDoubleValue();
      //angle = (angle * 180/Math.PI)%360;
      finale[0] = angle;
      finale[1] = (angleVel);
      return finale;

   }

   private double[] calculateAnglePosAndDerOfShoulderJointAcrobot(PinJoint joint)
   {
      double[] finale = new double[2];
      double firstAngle = 0;

      firstAngle = (robot.getLegJoint().getSecondJoint().getQ().getDoubleValue())%(Math.PI*2);

      if(firstAngle>Math.PI)
         firstAngle = (Math.PI*2-firstAngle)*-1;
      double angle = (joint.getQ().getDoubleValue())%(Math.PI*2)+firstAngle;
      if(angle > Math.PI)
         angle = angle - Math.PI*2;

      double angleVel = joint.getQD().getDoubleValue();

      finale[0] = angle;
      finale[1] = angleVel;
      return finale;
   }

   private double[] calculateAnglePosAndDerOfShoulderJointSkippy(PinJoint joint)
   {
      double[] finale = new double[2];
      double firstAngle = 0;

      firstAngle = (robot.getLegJoint().getSecondJoint().getQ().getDoubleValue() - Math.PI)%(Math.PI*2);

      if(firstAngle>Math.PI)
         firstAngle = (Math.PI*2-firstAngle)*-1;
      double angle = (joint.getQ().getDoubleValue())%(Math.PI*2)+firstAngle;
      if(angle > Math.PI)
         angle = angle - Math.PI*2;

      double angleVel = joint.getQD().getDoubleValue();

      finale[0] = angle;
      finale[1] = angleVel;
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
      positionJointsBasedOnError(robot.getHipJointAcrobot(), desiredPositions.get(timeCounter)[2], hipIntegralTerm, 20000, 150, 2000, false);
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
