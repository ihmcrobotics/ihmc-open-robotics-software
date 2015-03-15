package us.ihmc.atlas;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class AtlasInverseDynamicsCalculatorTestHelper
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   private final YoVariableRegistry registry = new YoVariableRegistry("AtlasInverseDynamicsCalculatorTestHelper");

   private final YoFrameVector computedRootJointForces = new YoFrameVector("tau_computed_root_force_", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector computedRootJointTorques = new YoFrameVector("tau_computed_root_torques_", ReferenceFrame.getWorldFrame(), registry);
   
   private final YoFrameVector leftFootComputedWrenchForce = new YoFrameVector("wrench_computed_leftFootForce", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameVector rightFootComputedWrenchForce = new YoFrameVector("wrench_computed_rightFootForce", ReferenceFrame.getWorldFrame(), registry);

   private final SDFRobot robot;
   private final SDFFullRobotModel fullRobotModel;
   private final SimulationConstructionSet scs;
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final LinkedHashMap<OneDoFJoint, DoubleYoVariable> computedJointTorques = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();

   public AtlasInverseDynamicsCalculatorTestHelper(boolean visualize)
   {
      boolean headless = false;
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasTarget.SIM, headless);
      fullRobotModel = atlasRobotModel.createFullRobotModel();
      
      boolean createCollisionMeshes = false;
      atlasRobotModel.setEnableJointDamping(false);
      robot = atlasRobotModel.createSdfRobot(createCollisionMeshes);
      
      List<GroundContactPoint> footGroundContactPoints = robot.getFootGroundContactPoints(RobotSide.LEFT);
      Joint leftAnkleJoint = footGroundContactPoints.get(0).getParentJoint();
      
      footGroundContactPoints = robot.getFootGroundContactPoints(RobotSide.RIGHT);
      Joint rightAnkleJoint = footGroundContactPoints.get(0).getParentJoint();
      
      IMUMount leftIMUMount = new IMUMount("leftIMU", new RigidBodyTransform(), robot);
      leftAnkleJoint.addIMUMount(leftIMUMount);
      IMUMount rightIMUMount = new IMUMount("rightIMU", new RigidBodyTransform(), robot);
      rightAnkleJoint.addIMUMount(rightIMUMount);
      
      atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasTarget.SIM, headless);
      
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         DoubleYoVariable computedJointTorque = new DoubleYoVariable("tau_computed_" + oneDegreeOfFreedomJoint.getName(), registry);
         computedJointTorques.put(oneDoFJoint, computedJointTorque);
      }

      twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
      double gravity = -robot.getGravityZ();
      inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravity);

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

   public boolean checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(double epsilon)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      SixDoFJoint rootJoint = fullRobotModel.getRootJoint();
      Wrench rootJointWrench = new Wrench(rootJoint.getFrameAfterJoint(), rootJoint.getFrameAfterJoint());
      rootJoint.packWrench(rootJointWrench);

      Vector3d rootJointForce = rootJointWrench.getLinearPart();
      Vector3d rootJointTorque = rootJointWrench.getAngularPart();

      computedRootJointForces.set(rootJointForce);
      computedRootJointTorques.set(rootJointTorque);

      boolean allTorquesMatch = true;

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         double inverseDynamicsTorque = oneDoFJoint.getTau();
         double simulatedRobotTorque = oneDegreeOfFreedomJoint.getTau().getDoubleValue();

         DoubleYoVariable computedJointTorque = computedJointTorques.get(oneDoFJoint);
         computedJointTorque.set(inverseDynamicsTorque);

         boolean torquesMatch = Math.abs(inverseDynamicsTorque - simulatedRobotTorque) < epsilon;
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

      SixDoFJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();

      setRootJointPositionAndOrientation(sixDoFJoint, floatingJoint);
      fullRobotModel.updateFrames();
      setRootJointVelocityAndAngularVelocity(sixDoFJoint, floatingJoint);


      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         oneDoFJoint.setQ(oneDegreeOfFreedomJoint.getQ().getDoubleValue());
         oneDoFJoint.setQd(oneDegreeOfFreedomJoint.getQD().getDoubleValue());
      }
   }

   public void setFullRobotModelAccelerationToMatchRobot()
   {
      robot.update();

      SixDoFJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();

      fullRobotModel.updateFrames();

      copyAccelerationFromForwardToInverse(floatingJoint, sixDoFJoint);

      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());
         oneDoFJoint.setQddDesired(oneDegreeOfFreedomJoint.getQDD().getDoubleValue());
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
            OneDegreeOfFreedomJoint joint = groundContactPointBasedWrenchCalculator.getJoint();
            OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(joint.getName());

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
   }

   public void setRootJointVelocityAndAngularVelocity(SixDoFJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      FrameVector angularVelocityFrameVector = new FrameVector();
      FrameVector linearVelocityFrameVector = new FrameVector();

      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      ReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      floatingJoint.getVelocity(linearVelocityFrameVector);
      linearVelocityFrameVector.changeFrame(bodyFrame);
      floatingJoint.getAngularVelocity(angularVelocityFrameVector, bodyFrame);

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, linearVelocityFrameVector.getVector(), angularVelocityFrameVector.getVector());
      sixDoFJoint.setJointTwist(bodyTwist);
   }


   public void setRootJointPositionAndOrientation(SixDoFJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      floatingJoint.getTransformToWorld(transformToWorld);
      sixDoFJoint.setPositionAndRotation(transformToWorld);
   }

   public void setRobotStateRandomly(Random random, double maxJointVelocity)
   {
      FloatingJoint rootJoint = robot.getRootJoint();
      rootJoint.setVelocity(RandomTools.generateRandomVector(random));
      rootJoint.setAngularVelocityInBody(RandomTools.generateRandomVector(random));

      rootJoint.setPosition(RandomTools.generateRandomVector(random));
      double yaw = RandomTools.generateRandomDouble(random, Math.PI / 20.0);
      double pitch = RandomTools.generateRandomDouble(random, Math.PI / 20.0);
      double roll = RandomTools.generateRandomDouble(random, Math.PI / 20.0);
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

         oneDegreeOfFreedomJoint.setQ(RandomTools.generateRandomDouble(random, lowerLimit, upperLimit));
         oneDegreeOfFreedomJoint.setQd(RandomTools.generateRandomDouble(random, maxJointVelocity));
      }
   }

   public void setRobotExternalForcesRandomly(Random random, double maxExternalForce)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         List<GroundContactPoint> footGroundContactPoints = robot.getFootGroundContactPoints(robotSide);

         for (GroundContactPoint groundContactPoint : footGroundContactPoints)
         {
            groundContactPoint.setForce(RandomTools.generateRandomVector(random, maxExternalForce));
         }
      }
   }

   public void setRobotTorquesRandomly(Random random)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         double maxTorque = 10.0;
         oneDegreeOfFreedomJoint.setTau(RandomTools.generateRandomDouble(random, maxTorque));
      }
   }

   public void copyAccelerationFromForwardToInverseBroken(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      SixDoFJointReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      FrameVector angularAccelerationInBody = new FrameVector();
      floatingJoint.getAngularAcceleration(angularAccelerationInBody, bodyFrame);
      FrameVector linearAccelerationInBody = new FrameVector();
      floatingJoint.getLinearAcceleration(linearAccelerationInBody);
      linearAccelerationInBody.changeFrame(bodyFrame);


      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);
      jointAcceleration.setLinearPart(linearAccelerationInBody);
      jointAcceleration.setAngularPart(angularAccelerationInBody);

      sixDoFJoint.setDesiredAcceleration(jointAcceleration);
   }

   public void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
   {
      // Note: To get the acceleration, you can't just changeFrame on the acceleration provided by SCS. Use setBasedOnOriginAcceleration instead.
      // TODO: Get this to work when the FloatingJoint has an offset.
      ReferenceFrame elevatorFrame = sixDoFJoint.getFrameBeforeJoint();
      SixDoFJointReferenceFrame bodyFrame = sixDoFJoint.getFrameAfterJoint();

      Twist bodyTwist = new Twist();
      sixDoFJoint.packJointTwist(bodyTwist);

      FrameVector originAcceleration = new FrameVector(elevatorFrame);
      FrameVector angularAcceleration = new FrameVector(bodyFrame);

      floatingJoint.getLinearAccelerationInWorld(originAcceleration.getVector());
      floatingJoint.getAngularAccelerationInBody(angularAcceleration.getVector());
      originAcceleration.changeFrame(elevatorFrame);

      SpatialAccelerationVector spatialAccelerationVector = new SpatialAccelerationVector(bodyFrame, elevatorFrame, bodyFrame);

      spatialAccelerationVector.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
      sixDoFJoint.setDesiredAcceleration(spatialAccelerationVector);
   }

   public void computeTwistCalculatorAndInverseDynamicsCalculator()
   {
      twistCalculator.compute();
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

   public SDFFullRobotModel getFullRobotModel()
   {
      return fullRobotModel;
   }
}
