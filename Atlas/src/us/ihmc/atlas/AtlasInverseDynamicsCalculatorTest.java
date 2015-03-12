package us.ihmc.atlas;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.atlas.AtlasRobotModel.AtlasTarget;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SixDoFJointReferenceFrame;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;

public class AtlasInverseDynamicsCalculatorTest
{
   private static final boolean VISUALIZE = false;

   @EstimatedDuration(duration = 0.5)
   @Test(timeout = 30000)
   public void testAtlasInverseDynamics() throws UnreasonableAccelerationException
   {
      SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      YoVariableRegistry registry = new YoVariableRegistry("AtlasInverseDynamicsCalculatorTest");

      Random random = new Random(1776L);

      boolean headless = false;
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.DRC_NO_HANDS, AtlasTarget.SIM, headless);


      SDFFullRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();
      SDFRobot robot = atlasRobotModel.createSdfRobotWithNoJointDamping();

      YoFrameVector computedRootJointForces = new YoFrameVector("tau_computed_root_force_", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector computedRootJointTorques = new YoFrameVector("tau_computed_root_torques_", ReferenceFrame.getWorldFrame(), registry);

      LinkedHashMap<OneDoFJoint, DoubleYoVariable> computedJointTorques = new LinkedHashMap<OneDoFJoint, DoubleYoVariable>();
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         DoubleYoVariable computedJointTorque = new DoubleYoVariable("tau_computed_" + oneDegreeOfFreedomJoint.getName(), registry);
         computedJointTorques.put(oneDoFJoint, computedJointTorque);
      }

      robot.addYoVariableRegistry(registry);

      SimulationConstructionSet scs = null;

      if (VISUALIZE)
      {
         scs = new SimulationConstructionSet(robot, simulationTestingParameters);
         double simulateDT = 0.00001;
         int recordFrequency = 1;
         scs.setDT(simulateDT, recordFrequency);
         scs.startOnAThread();
      }

      int numberOfTicks = 100;

      for (int i = 0; i < numberOfTicks; i++)
      {
         setRobotStateRandomly(random, robot);
         setRobotTorquesRandomly(random, robot);

         robot.doDynamicsButDoNotIntegrate();
         setFullRobotModelStateAndAccelerationToMatchRobot(fullRobotModel, robot);

         TwistCalculator twistCalculator = new TwistCalculator(ReferenceFrame.getWorldFrame(), fullRobotModel.getElevator());
         double gravity = -robot.getGravityZ();
         InverseDynamicsCalculator inverseDynamicsCalculator = new InverseDynamicsCalculator(twistCalculator, gravity);

         fullRobotModel.updateFrames();

         twistCalculator.compute();
         inverseDynamicsCalculator.reset();
         inverseDynamicsCalculator.compute();

         double epsilon = 1e-7;
         boolean torquesMatch = checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(computedRootJointForces, computedRootJointTorques,
                                   computedJointTorques, fullRobotModel, robot, epsilon);

         assertTrue(torquesMatch);

         assertEquals(0.0, computedRootJointForces.length(), 1e-10);
         assertEquals(0.0, computedRootJointTorques.length(), 1e-10);

         if (VISUALIZE)
            scs.tickAndUpdate();
      }


      if (VISUALIZE)
      {
         scs.gotoInPointNow();
         scs.tick(2);
         ThreadTools.sleep(100L);
         scs.setInPoint();
         ThreadTools.sleep(100L);
         scs.cropBuffer();

         if (simulationTestingParameters.getKeepSCSUp())
         {
            ThreadTools.sleepForever();
         }
      }
   }

   private boolean checkTorquesMatchBetweenFullRobotModelAndSimulatedRobot(YoFrameVector computedRootJointForces, YoFrameVector computedRootJointTorques,
           LinkedHashMap<OneDoFJoint, DoubleYoVariable> computedJointTorques, SDFFullRobotModel fullRobotModel, SDFRobot robot, double epsilon)
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

   private void setFullRobotModelStateAndAccelerationToMatchRobot(SDFFullRobotModel fullRobotModel, SDFRobot robot)
   {
      robot.update();

      SixDoFJoint sixDoFJoint = fullRobotModel.getRootJoint();
      FloatingJoint floatingJoint = robot.getRootJoint();

      setRootJointPositionAndOrientation(sixDoFJoint, floatingJoint);
      fullRobotModel.updateFrames();
      setRootJointVelocityAndAngularVelocity(sixDoFJoint, floatingJoint);

      copyAccelerationFromForwardToInverse(floatingJoint, sixDoFJoint);


      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(oneDegreeOfFreedomJoint.getName());

         oneDoFJoint.setQ(oneDegreeOfFreedomJoint.getQ().getDoubleValue());
         oneDoFJoint.setQd(oneDegreeOfFreedomJoint.getQD().getDoubleValue());
         oneDoFJoint.setQddDesired(oneDegreeOfFreedomJoint.getQDD().getDoubleValue());
      }
   }

   private void setRootJointVelocityAndAngularVelocity(SixDoFJoint sixDoFJoint, FloatingJoint floatingJoint)
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


   private void setRootJointPositionAndOrientation(SixDoFJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      floatingJoint.getTransformToWorld(transformToWorld);
      sixDoFJoint.setPositionAndRotation(transformToWorld);
   }

   private void setRootJointPositionAndOrientationTwo(SixDoFJoint sixDoFJoint, FloatingJoint floatingJoint)
   {
      Tuple3d position = new Vector3d();
      floatingJoint.getPosition(position);
      sixDoFJoint.setPosition(position);

      double[] yawPitchRoll = floatingJoint.getYawPitchRoll();
      sixDoFJoint.setRotation(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   private void setRobotStateRandomly(Random random, SDFRobot robot)
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
         double maxVelocity = 1.0;
         oneDegreeOfFreedomJoint.setQd(RandomTools.generateRandomDouble(random, maxVelocity));
      }
   }

   private void setRobotTorquesRandomly(Random random, SDFRobot robot)
   {
      ArrayList<OneDegreeOfFreedomJoint> oneDegreeOfFreedomJoints = new ArrayList<OneDegreeOfFreedomJoint>();
      robot.getAllOneDegreeOfFreedomJoints(oneDegreeOfFreedomJoints);

      for (OneDegreeOfFreedomJoint oneDegreeOfFreedomJoint : oneDegreeOfFreedomJoints)
      {
         double maxTorque = 10.0;
         oneDegreeOfFreedomJoint.setTau(RandomTools.generateRandomDouble(random, maxTorque));
      }
   }

   private void copyAccelerationFromForwardToInverseBroken(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
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

   private void copyAccelerationFromForwardToInverse(FloatingJoint floatingJoint, SixDoFJoint sixDoFJoint)
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

}
