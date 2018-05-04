package us.ihmc.avatar.obstacleCourseTests;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class DRCInverseDynamicsCalculatorTestHelper
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private final YoVariableRegistry registry = new YoVariableRegistry("InverseDynamicsCalculatorTestHelper");

   private final YoFrameVector3D computedRootJointForces = new YoFrameVector3D("tau_computed_root_force_", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D computedRootJointTorques = new YoFrameVector3D("tau_computed_root_torques_", ReferenceFrame.getWorldFrame(), registry);

   private final YoFrameVector3D leftFootComputedWrenchForce = new YoFrameVector3D("wrench_computed_leftFootForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D rightFootComputedWrenchForce = new YoFrameVector3D("wrench_computed_rightFootForce", ReferenceFrame.getWorldFrame(), registry);

   private final HumanoidFloatingRootJointRobot robot;
   private final FullHumanoidRobotModel fullRobotModel;
   private final SimulationConstructionSet scs;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final LinkedHashMap<OneDoFJoint, YoDouble> computedJointTorques = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> computedJointAccelerations = new LinkedHashMap<OneDoFJoint, YoDouble>();
   private final LinkedHashMap<OneDoFJoint, YoDouble> differenceJointTorques = new LinkedHashMap<OneDoFJoint, YoDouble>();

   private final YoFrameVector3D computedRootJointLinearAcceleration = new YoFrameVector3D("qdd_computed_root_linear", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector3D computedRootJointAngularAcceleration;

   private final ExternalForcePoint rootJointExternalForcePoint;

   private final SideDependentList<ExternalForcePoint> feetExternalForcePoints;

   public DRCInverseDynamicsCalculatorTestHelper(FullHumanoidRobotModel fullRobotModel, HumanoidFloatingRootJointRobot robot, boolean visualize, double gravityZ)
   {
      this.fullRobotModel = fullRobotModel;
      this.robot = robot;

      FloatingJoint rootJoint = robot.getRootJoint();
      rootJointExternalForcePoint = new ExternalForcePoint("rootJointExternalForce", robot);
      rootJoint.addExternalForcePoint(rootJointExternalForcePoint);

      List<GroundContactPoint> footGroundContactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      Joint leftAnkleJoint = footGroundContactPoints.get(0).getParentJoint();

      footGroundContactPoints = robot.getFootGroundContactPoints(RobotSide.RIGHT);
      Joint rightAnkleJoint = footGroundContactPoints.get(0).getParentJoint();

      IMUMount leftIMUMount = new IMUMount("leftIMU", new RigidBodyTransform(), robot);
      leftAnkleJoint.addIMUMount(leftIMUMount);
      IMUMount rightIMUMount = new IMUMount("rightIMU", new RigidBodyTransform(), robot);
      rightAnkleJoint.addIMUMount(rightIMUMount);

      ExternalForcePoint leftFootExternalForcePoint = new ExternalForcePoint("leftFootExternalForcePoint", robot);
      leftAnkleJoint.addExternalForcePoint(leftFootExternalForcePoint);

      ExternalForcePoint rightFootExternalForcePoint = new ExternalForcePoint("rightFootExternalForcePoint", robot);
      rightAnkleJoint.addExternalForcePoint(rightFootExternalForcePoint);

      feetExternalForcePoints = new SideDependentList<ExternalForcePoint>(leftFootExternalForcePoint, rightFootExternalForcePoint);

      computedRootJointAngularAcceleration = new YoFrameVector3D("qdd_computed_root_angular", fullRobotModel.getRootJoint().getFrameAfterJoint(), registry);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         YoDouble computedJointTorque = new YoDouble("tau_computed_" + oneDegreeOfFreedomJoint.getName(), registry);
         computedJointTorques.put(oneDoFJoint, computedJointTorque);

         YoDouble differenceJointTorque = new YoDouble("tau_difference_" + oneDegreeOfFreedomJoint.getName(), registry);
         differenceJointTorques.put(oneDoFJoint, differenceJointTorque);

         YoDouble computedJointAcceleration = new YoDouble("qdd_computed_" + oneDegreeOfFreedomJoint.getName(), registry);
         computedJointAccelerations.put(oneDoFJoint, computedJointAcceleration);
      }

      double gravity = -robot.getGravityZ();
      inverseDynamicsCalculator = new InverseDynamicsCalculator(fullRobotModel.getElevator(), gravity);

      robot.addYoVariableRegistry(registry);

      if (visualize)
      {
         scs = new SimulationConstructionSet(robot, simulationTestingParameters);
         double simulateDT = 0.00001;
         int recordFrequency = 1;
         scs.setDT(simulateDT, recordFrequency);
      }

      else
      {
         scs = null;
      }
   }

   public void startSimulationOnAThread()
   {
      if (scs != null)
         scs.startOnAThread();
   }

   public void setRobotTorquesToMatchFullRobotModel()
   {
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame bodyFixedFrame = fullRobotModel.getPelvis().getBodyFixedFrame();

      Wrench rootJointWrench = new Wrench(bodyFixedFrame, bodyFixedFrame);
      rootJoint.getWrench(rootJointWrench);

      FrameVector3D rootJointForce = rootJointWrench.getLinearPartAsFrameVectorCopy();
      FrameVector3D rootJointTorque = rootJointWrench.getAngularPartAsFrameVectorCopy();

      rootJointForce.changeFrame(ReferenceFrame.getWorldFrame());
      rootJointTorque.changeFrame(ReferenceFrame.getWorldFrame());

      computedRootJointForces.set(rootJointForce);
      computedRootJointTorques.set(rootJointTorque);

      rootJointExternalForcePoint.setForce(new Vector3D(rootJointForce));
      rootJointExternalForcePoint.setMoment(new Vector3D(rootJointTorque));

      FramePoint3D rootJointPosition = new FramePoint3D(bodyFixedFrame);
      rootJointPosition.changeFrame(ReferenceFrame.getWorldFrame());

      rootJointExternalForcePoint.setOffsetWorld(rootJointPosition);
      Vector3D offsetInJoint = rootJointExternalForcePoint.getOffsetCopy();

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         double inverseDynamicsTorque = oneDoFJoint.getTau();
         oneDegreeOfFreedomJoint.setTau(inverseDynamicsTorque);

         YoDouble computedJointTorque = computedJointTorques.get(oneDoFJoint);
         computedJointTorque.set(inverseDynamicsTorque);
      }
   }

   public void setRobotTorquesToMatchFullRobotModelButCheckAgainstOtherRobot(FloatingRootJointRobot otherRobot, double epsilon)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      ArrayList<OneDegreeOfFreedomJoint> otherOneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      otherRobot.getAllOneDegreeOfFreedomJoints(otherOneDegreeOfFreedomJoints);

      for (int i = 0; i < oneDegreeOfFreedomJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = oneDegreeOfFreedomJoints.get(i);
         OneDegreeOfFreedomJoint otherOneDegreeOfFreedomJoint = otherOneDegreeOfFreedomJoints.get(i);

         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         double inverseDynamicsTorque = oneDoFJoint.getTau();
         oneDegreeOfFreedomJoint.setTau(inverseDynamicsTorque);

         YoDouble computedJointTorque = computedJointTorques.get(oneDoFJoint);
         computedJointTorque.set(inverseDynamicsTorque);

         double otherRobotJointTorque = otherOneDegreeOfFreedomJoint.getTauYoVariable().getDoubleValue();

         YoDouble differenceJointTorque = differenceJointTorques.get(oneDoFJoint);
         differenceJointTorque.set(otherRobotJointTorque - inverseDynamicsTorque);
         if (Math.abs(differenceJointTorque.getDoubleValue()) > epsilon)
         {
            System.err.println("Torques don't match. oneDegreeOfFreedomJoint = " + oneDegreeOfFreedomJoint.getName() + ", otherRobotJointTorque = "
                  + otherRobotJointTorque + ", inverseDynamicsTorque = " + inverseDynamicsTorque);
         }
      }
   }

   public void setRobotTorquesToMatchOtherRobot(FloatingRootJointRobot otherRobot)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      ArrayList<OneDegreeOfFreedomJoint> otherOneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      otherRobot.getAllOneDegreeOfFreedomJoints(otherOneDegreeOfFreedomJoints);

      for (int i = 0; i < oneDegreeOfFreedomJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = oneDegreeOfFreedomJoints.get(i);
         OneDegreeOfFreedomJoint otherOneDegreeOfFreedomJoint = otherOneDegreeOfFreedomJoints.get(i);

         if (!oneDegreeOfFreedomJoint.getName().equals(otherOneDegreeOfFreedomJoint.getName()))
         {
            throw new RuntimeException();
         }

         oneDegreeOfFreedomJoint.setTau(otherOneDegreeOfFreedomJoint.getTauYoVariable().getDoubleValue());
      }
   }

   public boolean checkAccelerationsMatchBetweenFullRobotModelAndSimulatedRobot(double epsilon)
   {
      FloatingInverseDynamicsJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();
      boolean allAccelerationsMatch = checkFullRobotModelRootJointAccelerationmatchesRobot(floatingJoint, sixDoFJoint, epsilon);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         double inverseDynamicsAcceleration = oneDoFJoint.getQddDesired();
         double simulatedRobotAcceleration = oneDegreeOfFreedomJoint.getQDDYoVariable().getDoubleValue();

         YoDouble computedJointAcceleration = computedJointAccelerations.get(oneDoFJoint);
         computedJointAcceleration.set(inverseDynamicsAcceleration);

         boolean accelerationsMatch = Math.abs(inverseDynamicsAcceleration - simulatedRobotAcceleration) < epsilon;
         if (!accelerationsMatch)
         {
            System.err.println("Accelerations don't match. oneDegreeOfFreedomJoint = " + oneDegreeOfFreedomJoint.getName() + " inverseDynamicsAcceleration = "
                  + inverseDynamicsAcceleration + ", simulatedRobotAcceleration = " + simulatedRobotAcceleration);
            allAccelerationsMatch = false;
         }
      }

      return allAccelerationsMatch;
   }

   public boolean checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(double epsilon)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      Wrench rootJointWrench = new Wrench(rootJoint.getFrameAfterJoint(), rootJoint.getFrameAfterJoint());
      rootJoint.getWrench(rootJointWrench);

      FrameVector3D rootJointForce = rootJointWrench.getLinearPartAsFrameVectorCopy();
      FrameVector3D rootJointTorque = rootJointWrench.getAngularPartAsFrameVectorCopy();

      rootJointForce.changeFrame(ReferenceFrame.getWorldFrame());
      rootJointTorque.changeFrame(ReferenceFrame.getWorldFrame());

      computedRootJointForces.set(rootJointForce);
      computedRootJointTorques.set(rootJointTorque);

      Vector3D simulatedRootJointForce = new Vector3D();
      Vector3D simulatedRootJointTorque = new Vector3D();

      rootJointExternalForcePoint.getForce(simulatedRootJointForce);
      rootJointExternalForcePoint.getMoment(simulatedRootJointTorque);

      Vector3D forceErrorVector = new Vector3D(rootJointForce);
      forceErrorVector.sub(simulatedRootJointForce);
      double rootJointForceError = forceErrorVector.length();

      Vector3D torqueErrorVector = new Vector3D(rootJointTorque);
      torqueErrorVector.sub(simulatedRootJointTorque);
      double rootJointTorqueError = forceErrorVector.length();

      boolean allTorquesMatch = ((rootJointForceError < epsilon) && (rootJointTorqueError < epsilon));
      if (!allTorquesMatch)
      {
         System.err.println("rootJointForceError = " + rootJointForceError + ", rootJointTorqueError = " + rootJointTorqueError);
      }

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         double inverseDynamicsTorque = oneDoFJoint.getTau();
         double simulatedRobotTorque = oneDegreeOfFreedomJoint.getTauYoVariable().getDoubleValue();

         YoDouble computedJointTorque = computedJointTorques.get(oneDoFJoint);
         computedJointTorque.set(inverseDynamicsTorque);

         boolean torquesMatch = Math.abs(inverseDynamicsTorque - simulatedRobotTorque) < epsilon;
         if (!torquesMatch)
         {
            System.err.println("Torques do not match. oneDegreeOfFreedomJoint = " + oneDegreeOfFreedomJoint.getName() + ", inverseDynamicsTorque = "
                  + inverseDynamicsTorque + ", simulatedRobotTorque = " + simulatedRobotTorque);
         }
         if (!torquesMatch)
            allTorquesMatch = false;
      }

      return allTorquesMatch;
   }

   public boolean checkComputedRootJointWrenchIsZero(double epsilon)
   {
      if (computedRootJointForces.length() > epsilon)
         return false;
      if (computedRootJointTorques.length() > epsilon)
         return false;

      return true;
   }

   public void setFullRobotModelStateAndAccelerationToMatchRobot()
   {
      setFullRobotModelStateToMatchRobot();
      setFullRobotModelAccelerationToMatchRobot();
   }

   public void setFullRobotModelStateToMatchRobot()
   {
      robot.update();

      FloatingInverseDynamicsJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();

      setFullRobotModelRootJointPositionAndOrientationToMatchRobot(sixDoFJoint, floatingJoint);
      fullRobotModel.updateFrames();
      setFullRobotModelRootJointVelocityAndAngularVelocityToMatchRobot(sixDoFJoint, floatingJoint);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         oneDoFJoint.setQ(oneDegreeOfFreedomJoint.getQYoVariable().getDoubleValue());
         oneDoFJoint.setQd(oneDegreeOfFreedomJoint.getQDYoVariable().getDoubleValue());
      }
   }

   public void setRobotStateToMatchOtherRobot(FloatingRootJointRobot otherRobot)
   {
      otherRobot.update();

      FloatingJoint floatingJoint = robot.getRootJoint();
      FloatingJoint otherFloatingJoint = otherRobot.getRootJoint();

      Tuple3DBasics position = new Point3D();
      Tuple3DBasics velocity = new Vector3D();
      otherFloatingJoint.getPositionAndVelocity(position, velocity);
      floatingJoint.setPositionAndVelocity(position, velocity);

      Quaternion rotation = new Quaternion();
      otherFloatingJoint.getQuaternion(rotation);
      floatingJoint.setQuaternion(rotation);

      Vector3D angularVelocityInBody = otherFloatingJoint.getAngularVelocityInBody();
      floatingJoint.setAngularVelocityInBody(angularVelocityInBody);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      ArrayList<OneDegreeOfFreedomJoint> otherOneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      otherRobot.getAllOneDegreeOfFreedomJoints(otherOneDegreeOfFreedomJoints);

      for (int i = 0; i < oneDegreeOfFreedomJoints.size(); i++)
      {
         OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint = oneDegreeOfFreedomJoints.get(i);
         OneDegreeOfFreedomJoint otherOneDegreeOfFreedomJoint = otherOneDegreeOfFreedomJoints.get(i);

         oneDegreeOfFreedomJoint.setQ(otherOneDegreeOfFreedomJoint.getQYoVariable().getDoubleValue());
         oneDegreeOfFreedomJoint.setQd(otherOneDegreeOfFreedomJoint.getQDYoVariable().getDoubleValue());
      }

   }

   public void setRobotStateToMatchFullRobotModel()
   {
      FloatingInverseDynamicsJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();

      fullRobotModel.updateFrames();

      setRobotRootJointPositionAndOrientationToMatchFullRobotModel(sixDoFJoint, floatingJoint);
      setRobotRootJointVelocityAndAngularVelocityToMatchFullRobotModel(sixDoFJoint, floatingJoint);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         oneDegreeOfFreedomJoint.setQ(oneDoFJoint.getQ());
         oneDegreeOfFreedomJoint.setQd(oneDoFJoint.getQd());
      }

      robot.update();
   }

   public void setFullRobotModelAccelerationRandomly(Random random, double maxPelvisLinearAcceleration, double maxPelvisAngularAcceleration,
         double maxJointAcceleration)
   {
      FloatingInverseDynamicsJoint sixDoFJoint = fullRobotModel.getRootJoint();
      setSixDoFJointAccelerationRandomly(sixDoFJoint, random, maxPelvisLinearAcceleration, maxPelvisAngularAcceleration);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());
         oneDoFJoint.setQddDesired(RandomNumbers.nextDouble(random, maxJointAcceleration));
      }
   }

   public void setFullRobotModelAccelerationToMatchRobot()
   {
      robot.update();

      FloatingInverseDynamicsJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();

      fullRobotModel.updateFrames();

      copyAccelerationFromForwardToInverse(floatingJoint, sixDoFJoint);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         double robotJointAcceleration = oneDegreeOfFreedomJoint.getQDDYoVariable().getDoubleValue();
         oneDoFJoint.setQddDesired(robotJointAcceleration);

         YoDouble computedJointAcceleration = computedJointAccelerations.get(oneDoFJoint);
         computedJointAcceleration.set(robotJointAcceleration);
      }
   }

   public void setFullRobotModelWrenchesToMatchRobot()
   {
      inverseDynamicsCalculator.reset();

      ArrayList<WrenchCalculatorInterface> groundContactPointBasedWrenchCalculators = new ArrayList<WrenchCalculatorInterface>();
      robot.getForceSensors(groundContactPointBasedWrenchCalculators);

      for (WrenchCalculatorInterface groundContactPointBasedWrenchCalculator : groundContactPointBasedWrenchCalculators)
      {
         if (groundContactPointBasedWrenchCalculator instanceof GroundContactPointBasedWrenchCalculator)
         {
            Joint joint = groundContactPointBasedWrenchCalculator.getJoint();
            OneDoFJoint oneDoFJoint;
            if (joint instanceof OneDegreeOfFreedomJoint)
               oneDoFJoint = fullRobotModel.getOneDoFJointByName(joint.getName());
            else
               throw new RuntimeException("Force sensor is only supported for OneDegreeOfFreedomJoint.");

            RigidBody rigidBodyToApplyWrenchTo = oneDoFJoint.getSuccessor();
            ReferenceFrame bodyFixedFrame = rigidBodyToApplyWrenchTo.getBodyFixedFrame();

            groundContactPointBasedWrenchCalculator.calculate();
            DenseMatrix64F wrenchFromSimulation = groundContactPointBasedWrenchCalculator.getWrench();
            ReferenceFrame frameAtJoint = rigidBodyToApplyWrenchTo.getParentJoint().getFrameAfterJoint();

            Wrench wrench = new Wrench(frameAtJoint, frameAtJoint, wrenchFromSimulation);
            wrench.changeBodyFrameAttachedToSameBody(bodyFixedFrame);
            wrench.changeFrame(bodyFixedFrame);

            inverseDynamicsCalculator.setExternalWrench(rigidBodyToApplyWrenchTo, wrench);
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);

         Wrench wrench = new Wrench();
         inverseDynamicsCalculator.getExternalWrench(foot, wrench);

         ReferenceFrame footFrame = foot.getBodyFixedFrame();

         ExternalForcePoint footExternalForcePoint = feetExternalForcePoints.get(robotSide);

         Vector3D externalForce = new Vector3D();
         Vector3D externalMoment = new Vector3D();

         footExternalForcePoint.getForce(externalForce);
         footExternalForcePoint.getMoment(externalMoment);
         //         System.out.println("externalMoment = " + externalMoment);

         FrameVector3D externalForcePointForce = new FrameVector3D(ReferenceFrame.getWorldFrame(), externalForce);
         FrameVector3D externalForcePointMoment = new FrameVector3D(ReferenceFrame.getWorldFrame(), externalMoment);

         externalForcePointForce.changeFrame(footFrame);
         externalForcePointMoment.changeFrame(footFrame);

         Point3D position = new Point3D();
         footExternalForcePoint.getPosition(position);
         FramePoint3D pointOfApplication = new FramePoint3D(ReferenceFrame.getWorldFrame(), position);
         pointOfApplication.changeFrame(footFrame);

         Vector3D torqueFromLeverArm = new Vector3D();
         Vector3D leverArm = new Vector3D(pointOfApplication);
         torqueFromLeverArm.cross(leverArm, externalForcePointForce);

         externalForcePointMoment.add(torqueFromLeverArm);

         FrameVector3D totalTorqueOnFoot = wrench.getAngularPartAsFrameVectorCopy();
         FrameVector3D totalForceOnFoot = wrench.getLinearPartAsFrameVectorCopy();

         totalTorqueOnFoot.add(externalForcePointMoment);
         totalForceOnFoot.add(externalForcePointForce);

         wrench.setAngularPart(totalTorqueOnFoot);
         wrench.setLinearPart(totalForceOnFoot);

         inverseDynamicsCalculator.setExternalWrench(foot, wrench);
      }
   }

   public void setFullRobotModelRootJointVelocityAndAngularVelocityToMatchRobot(FloatingInverseDynamicsJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      FrameVector3D angularVelocityFrameVector = new FrameVector3D();
      FrameVector3D linearVelocityFrameVector = new FrameVector3D();

      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      floatingJoint.getVelocity(linearVelocityFrameVector);
      linearVelocityFrameVector.changeFrame(bodyFrame);
      floatingJoint.getAngularVelocity(angularVelocityFrameVector, bodyFrame);

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, linearVelocityFrameVector, angularVelocityFrameVector);
      sixDoFJoint.setJointTwist(bodyTwist);
   }

   public void setRobotRootJointVelocityAndAngularVelocityToMatchFullRobotModel(FloatingInverseDynamicsJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      Twist rootJointTwist = new Twist();
      sixDoFJoint.getJointTwist(rootJointTwist);

      floatingJoint.setAngularVelocityInBody(rootJointTwist.getAngularPartCopy());

      FrameVector3D linearVelocityInWorld = new FrameVector3D();
      rootJointTwist.getLinearPart(linearVelocityInWorld);

      linearVelocityInWorld.changeFrame(ReferenceFrame.getWorldFrame());
      floatingJoint.setVelocity(new Vector3D(linearVelocityInWorld));
   }

   public void setFullRobotModelRootJointPositionAndOrientationToMatchRobot(FloatingInverseDynamicsJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      floatingJoint.getTransformToWorld(transformToWorld);
      sixDoFJoint.setPositionAndRotation(transformToWorld);
   }

   public void setRobotRootJointPositionAndOrientationToMatchFullRobotModel(FloatingInverseDynamicsJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      RigidBodyTransform transform = new RigidBodyTransform(sixDoFJoint.getJointTransform3D());
      floatingJoint.setRotationAndTranslation(transform);
   }

   public void setFullRobotModelStateRandomly(Random random, double maxJointVelocity, double maxRootJointLinearAndAngularVelocity)
   {
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();

      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = rootJoint.getFrameAfterJoint();

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, RandomGeometry.nextVector3D(random, maxRootJointLinearAndAngularVelocity),
            RandomGeometry.nextVector3D(random, maxRootJointLinearAndAngularVelocity));
      rootJoint.setJointTwist(bodyTwist);

      rootJoint.setPosition(RandomGeometry.nextVector3D(random));

      double yaw = RandomNumbers.nextDouble(random, Math.PI / 20.0);
      double pitch = RandomNumbers.nextDouble(random, Math.PI / 20.0);
      double roll = RandomNumbers.nextDouble(random, Math.PI / 20.0);
      rootJoint.setRotation(yaw, pitch, roll);

      ArrayList<OneDoFJoint> oneDoFJoints = new ArrayList<OneDoFJoint>();
      fullRobotModel.getOneDoFJoints(oneDoFJoints);

      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         double lowerLimit = oneDoFJoint.getJointLimitLower();
         double upperLimit = oneDoFJoint.getJointLimitUpper();
         double delta = upperLimit - lowerLimit;
         lowerLimit = lowerLimit + 0.05 * delta;
         upperLimit = upperLimit - 0.05 * delta;

         oneDoFJoint.setQ(RandomNumbers.nextDouble(random, lowerLimit, upperLimit));
         oneDoFJoint.setQd(RandomNumbers.nextDouble(random, maxJointVelocity));
      }
   }

   public void setRobotStateRandomly(Random random, double maxJointVelocity, double maxRootJointLinearAndAngularVelocity)
   {
      FloatingJoint rootJoint = robot.getRootJoint();
      rootJoint.setVelocity(RandomGeometry.nextVector3D(random, maxRootJointLinearAndAngularVelocity));
      rootJoint.setAngularVelocityInBody(RandomGeometry.nextVector3D(random, maxRootJointLinearAndAngularVelocity));

      rootJoint.setPosition(RandomGeometry.nextVector3D(random));
      double yaw = RandomNumbers.nextDouble(random, Math.PI / 20.0);
      double pitch = RandomNumbers.nextDouble(random, Math.PI / 20.0);
      double roll = RandomNumbers.nextDouble(random, Math.PI / 20.0);
      rootJoint.setYawPitchRoll(yaw, pitch, roll);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         double lowerLimit = oneDegreeOfFreedomJoint.getJointLowerLimit();
         double upperLimit = oneDegreeOfFreedomJoint.getJointUpperLimit();
         double delta = upperLimit - lowerLimit;
         lowerLimit = lowerLimit + 0.05 * delta;
         upperLimit = upperLimit - 0.05 * delta;

         oneDegreeOfFreedomJoint.setQ(RandomNumbers.nextDouble(random, lowerLimit, upperLimit));
         oneDegreeOfFreedomJoint.setQd(RandomNumbers.nextDouble(random, maxJointVelocity));
      }
   }

   public void setRobotExternalForcesRandomly(Random random, double maxGroundContactPointForce, double maxFootExternalForce, double maxFootExternalTorque)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         List<GroundContactPoint> footGroundContactPoints = robot.getFootGroundContactPoints(robotSide);

         for (GroundContactPoint groundContactPoint : footGroundContactPoints)
         {
            groundContactPoint.setForce(RandomGeometry.nextVector3D(random, maxGroundContactPointForce));
         }

         ExternalForcePoint footExternalForcePoint = feetExternalForcePoints.get(robotSide);
         footExternalForcePoint.setForce(RandomGeometry.nextVector3D(random, maxFootExternalForce));
         footExternalForcePoint.setMoment(RandomGeometry.nextVector3D(random, maxFootExternalTorque));
      }
   }

   public void setRobotsExternalForcesToMatchFullRobotModel()
   {
      setRobotsExternalForcesToMatchFullRobotModel(inverseDynamicsCalculator);
   }

   public void setRobotsExternalForcesToMatchOtherRobot(FloatingRootJointRobot otherRobot)
   {
      ArrayList<GroundContactPoint> otherGroundContactPoints = otherRobot.getAllGroundContactPoints();
      ArrayList<GroundContactPoint> groundContactPoints = robot.getAllGroundContactPoints();

      for (int i = 0; i < otherGroundContactPoints.size(); i++)
      {
         GroundContactPoint otherGroundContactPoint = otherGroundContactPoints.get(i);
         GroundContactPoint groundContactPoint = groundContactPoints.get(i);

         Vector3D force = new Vector3D();
         otherGroundContactPoint.getForce(force);
         groundContactPoint.setForce(force);

         Vector3D moment = new Vector3D();
         otherGroundContactPoint.getMoment(moment);
         groundContactPoint.setMoment(moment);

         boolean inContact = otherGroundContactPoint.isInContact();
         groundContactPoint.setIsInContact(inContact);
      }

   }

   public void setRobotsExternalForcesToMatchFullRobotModel(InverseDynamicsCalculator inverseDynamicsCalculator)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);

         Wrench wrench = new Wrench();
         inverseDynamicsCalculator.getExternalWrench(foot, wrench);

         ReferenceFrame bodyFixedFrame = foot.getBodyFixedFrame();
         FramePoint3D pointOfWrenchApplication = new FramePoint3D(bodyFixedFrame);
         pointOfWrenchApplication.changeFrame(ReferenceFrame.getWorldFrame());

         ExternalForcePoint footExternalForcePoint = feetExternalForcePoints.get(robotSide);
         footExternalForcePoint.setOffsetWorld(pointOfWrenchApplication);

         FrameVector3D wrenchForce = wrench.getLinearPartAsFrameVectorCopy();
         wrenchForce.changeFrame(ReferenceFrame.getWorldFrame());

         FrameVector3D wrenchTorque = wrench.getAngularPartAsFrameVectorCopy();
         wrenchTorque.changeFrame(ReferenceFrame.getWorldFrame());

         footExternalForcePoint.setForce(new Vector3D(wrenchForce));
         footExternalForcePoint.setMoment(new Vector3D(wrenchTorque));
      }
   }

   public void setFullRobotModelExternalForcesRandomly(Random random, double maxFeetExternalForce, double maxFeetExternalTorque)
   {
      inverseDynamicsCalculator.reset();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         ReferenceFrame bodyFixedFrame = foot.getBodyFixedFrame();

         Wrench wrench = new Wrench(bodyFixedFrame, bodyFixedFrame, RandomGeometry.nextVector3D(random, maxFeetExternalForce),
               RandomGeometry.nextVector3D(random, maxFeetExternalTorque));
         inverseDynamicsCalculator.setExternalWrench(foot, wrench);
      }
   }

   public void setRobotRootJointExternalForcesRandomly(Random random, double maxRootJointForceAndTorque)
   {
      rootJointExternalForcePoint.setForce(RandomGeometry.nextVector3D(random, maxRootJointForceAndTorque));
      rootJointExternalForcePoint.setMoment(RandomGeometry.nextVector3D(random, maxRootJointForceAndTorque));
   }

   public void setRobotTorquesRandomly(Random random, double maxJointTorque)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         oneDegreeOfFreedomJoint.setTau(RandomNumbers.nextDouble(random, maxJointTorque));
      }
   }

   public void copyAccelerationFromForwardToInverseBroken(FloatingJoint floatingJoint, FloatingInverseDynamicsJoint sixDoFJoint)
   {
      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      FrameVector3D angularAccelerationInBody = new FrameVector3D();
      floatingJoint.getAngularAcceleration(angularAccelerationInBody, bodyFrame);
      FrameVector3D linearAccelerationInBody = new FrameVector3D();
      floatingJoint.getLinearAcceleration(linearAccelerationInBody);
      linearAccelerationInBody.changeFrame(bodyFrame);

      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);
      jointAcceleration.setLinearPart(linearAccelerationInBody);
      jointAcceleration.setAngularPart(angularAccelerationInBody);

      sixDoFJoint.setDesiredAcceleration(jointAcceleration);
   }

   public void setSixDoFJointAccelerationRandomly(FloatingInverseDynamicsJoint sixDoFJoint, Random random, double maxRootJointLinearAcceleration,
         double maxRootJointAngularAcceleration)
   {
      // Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use setBasedOnOriginAcceleration instead.
      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      Twist bodyTwist = new Twist();
      sixDoFJoint.getJointTwist(bodyTwist);

      FrameVector3D originAcceleration = new FrameVector3D(elevatorFrame, RandomGeometry.nextVector3D(random, maxRootJointLinearAcceleration));
      FrameVector3D angularAcceleration = new FrameVector3D(bodyFrame, RandomGeometry.nextVector3D(random, maxRootJointAngularAcceleration));
      //      originAcceleration.changeFrame(elevatorFrame);

      SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);

      spatialAccelerationVector.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
      sixDoFJoint.setDesiredAcceleration(spatialAccelerationVector);
   }

   public void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, FloatingInverseDynamicsJoint sixDoFJoint)
   {
      // Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use setBasedOnOriginAcceleration instead.
      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      Twist bodyTwist = new Twist();
      sixDoFJoint.getJointTwist(bodyTwist);

      FrameVector3D originAcceleration = new FrameVector3D(elevatorFrame);
      FrameVector3D angularAcceleration = new FrameVector3D(bodyFrame);

      floatingJoint.getLinearAccelerationInWorld(originAcceleration);
      floatingJoint.getAngularAccelerationInBody(angularAcceleration);
      originAcceleration.changeFrame(elevatorFrame);

      SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);

      spatialAccelerationVector.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
      sixDoFJoint.setDesiredAcceleration(spatialAccelerationVector);
   }

   public boolean checkFullRobotModelRootJointAccelerationmatchesRobot(FloatingJoint floatingJoint, FloatingInverseDynamicsJoint sixDoFJoint, double epsilon)
   {
      // Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use setBasedOnOriginAcceleration instead.
      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      Twist bodyTwist = new Twist();
      sixDoFJoint.getJointTwist(bodyTwist);

      FrameVector3D originAcceleration = new FrameVector3D(elevatorFrame);
      FrameVector3D angularAcceleration = new FrameVector3D(bodyFrame);

      floatingJoint.getLinearAccelerationInWorld(originAcceleration);
      floatingJoint.getAngularAccelerationInBody(angularAcceleration);
      originAcceleration.changeFrame(elevatorFrame);

      //TODO: These should be from the inverse dynamics to the SCS frames...
      originAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
      computedRootJointLinearAcceleration.set(originAcceleration);
      computedRootJointAngularAcceleration.set(angularAcceleration);

      SpatialAccelerationVector spatialAccelerationVectorOfSimulatedRootJoint = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);
      spatialAccelerationVectorOfSimulatedRootJoint.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);

      SpatialAccelerationVector spatialAccelerationVectorOfInverseDynamicsRootJoint = new SpatialAccelerationVector();
      sixDoFJoint.getDesiredJointAcceleration(spatialAccelerationVectorOfInverseDynamicsRootJoint);

      return spatialAccelerationVectorOfInverseDynamicsRootJoint.epsilonEquals(spatialAccelerationVectorOfInverseDynamicsRootJoint, epsilon);
   }

   public void computeTwistCalculatorAndInverseDynamicsCalculator()
   {
      inverseDynamicsCalculator.compute();
   }

   public Robot getRobot()
   {
      return robot;
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return scs;
   }

   public SimulationTestingParameters getSimulationTestingParameters()
   {
      return simulationTestingParameters;
   }

   public FullHumanoidRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }

}
