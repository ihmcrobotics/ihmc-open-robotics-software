package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.stream.Collectors;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.SpatialVector;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class AvatarEndToEndForwardDynamicsCalculatorTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testStanding() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.createSimulation("FwdDynamicAgainstSCS_Standing");
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      ForwardDynamicComparisonScript script = new ForwardDynamicComparisonScript(robot, robotModel);
      drcSimulationTestHelper.getSimulationConstructionSet().addScript(script);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));
      script.setPerformAssertions(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0));
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testWalking() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);
      drcSimulationTestHelper.setAddFootstepMessageGenerator(true);
      drcSimulationTestHelper.setUseHeadingAndVelocityScript(true);
      drcSimulationTestHelper.createSimulation("FwdDynamicAgainstSCS_Walking");
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      ForwardDynamicComparisonScript script = new ForwardDynamicComparisonScript(robot, robotModel);
      drcSimulationTestHelper.getSimulationConstructionSet().addScript(script);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      drcSimulationTestHelper.getSimulationConstructionSet().findVariable("walkCSG").setValueFromDouble(1.0);
      script.setPerformAssertions(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(10.0));
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testFloating() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCRobotModel robotModel = getRobotModel();

      CommonAvatarEnvironmentInterface environment = new CommonAvatarEnvironmentInterface()
      {
         @Override
         public TerrainObject3D getTerrainObject3D()
         {
            return new CombinedTerrainObject3D("Space");
         }

         @Override
         public List<? extends Robot> getEnvironmentRobots()
         {
            return null;
         }

         @Override
         public void createAndSetContactControllerToARobot()
         {
         }

         @Override
         public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
         {
         }

         @Override
         public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
         {
         }
      };

      Random random = new Random(543543);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.getSimulationStarter().registerControllerStateTransition(new ControllerStateTransitionFactory<HighLevelControllerName>()
      {
         @Override
         public HighLevelControllerName getStateToAttachEnum()
         {
            return HighLevelControllerName.WALKING;
         }

         @Override
         public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> stateMap,
                                                                                    HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                                    YoRegistry parentRegistry)
         {
            return new StateTransition<>(HighLevelControllerName.DO_NOTHING_BEHAVIOR, t -> true);
         }
      });
      drcSimulationTestHelper.createSimulation("FwdDynamicAgainstSCS_Floating");
      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      HumanoidFloatingRootJointRobot robot = drcSimulationTestHelper.getRobot();
      robot.setGravity(0.0);
      ForwardDynamicComparisonScript script = new ForwardDynamicComparisonScript(robot, robotModel);
      scs.addScript(script);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5));
      script.setPerformAssertions(true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0));
      setRandomConfiguration(random, robot);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      robot.setGravity(-9.81);
      setRandomConfiguration(random, robot);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      robot.setGravity(0.0);
      setRandomConfiguration(random, robot);
      setRandomVelocity(random, robot);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));
      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   public static void setRandomConfiguration(Random random, Robot robot)
   {
      FloatingJoint rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
      rootJoint.setRotationAndTranslation(EuclidCoreRandomTools.nextRigidBodyTransform(random));

      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(joints);
      for (OneDegreeOfFreedomJoint joint : joints)
      {
         double jointLowerLimit = Math.max(joint.getJointLowerLimit(), -2.0 * Math.PI);
         double jointUpperLimit = Math.min(joint.getJointUpperLimit(), 2.0 * Math.PI);
         double q = EuclidCoreRandomTools.nextDouble(random, jointLowerLimit, jointUpperLimit);
         joint.setQ(q);
      }
   }

   public static void setRandomVelocity(Random random, Robot robot)
   {
      FloatingJoint rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
      rootJoint.setVelocity(EuclidCoreRandomTools.nextVector3D(random));
      rootJoint.setAngularVelocityInBody(EuclidCoreRandomTools.nextVector3D(random));

      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(joints);
      for (OneDegreeOfFreedomJoint joint : joints)
         joint.setQd(EuclidCoreRandomTools.nextDouble(random));
   }

   public static void setRandomEffort(Random random, Robot robot)
   {
      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
      robot.getAllOneDegreeOfFreedomJoints(joints);
      for (OneDegreeOfFreedomJoint joint : joints)
         joint.setTau(EuclidCoreRandomTools.nextDouble(random));
   }

   public static class ForwardDynamicComparisonScript implements Script
   {
      private final Robot robot;
      private final ForwardDynamicsCalculator calculator;
      private final MultiBodySystemBasics multiBodySystem;

      private final FloatingJointBasics floatingJoint;
      private final List<OneDoFJointBasics> allOneDoFJoints;
      private final Map<String, JointBasics> nameToJointMap = new LinkedHashMap<>();

      private boolean performAssertions = false;

      public ForwardDynamicComparisonScript(Robot robot, DRCRobotModel robotModel)
      {
         this.robot = robot;
         FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();

         List<JointBasics> ignoredJoints = robotModel.getJointMap().getLastSimulatedJoints().stream()
                                                     .flatMap(name -> SubtreeStreams.fromChildren(fullRobotModel.getOneDoFJointByName(name).getSuccessor()))
                                                     .collect(Collectors.toList());

         multiBodySystem = MultiBodySystemBasics.toMultiBodySystemBasics(fullRobotModel.getElevator(), ignoredJoints);
         calculator = new ForwardDynamicsCalculator(multiBodySystem);

         floatingJoint = fullRobotModel.getRootJoint();
         allOneDoFJoints = MultiBodySystemTools.filterJoints(multiBodySystem.getJointsToConsider(), OneDoFJointBasics.class);

         multiBodySystem.getAllJoints().forEach(joint -> nameToJointMap.put(joint.getName(), joint));
      }

      public void setPerformAssertions(boolean performAssertions)
      {
         this.performAssertions = performAssertions;
      }

      @Override
      public void doScript(double t)
      {
         try
         {
            robot.doDynamicsButDoNotIntegrate();
         }
         catch (UnreasonableAccelerationException e)
         {
            e.printStackTrace();
         }

         calculator.setGravitionalAcceleration(robot.getGravityZ());
         getFloatingJointStateFromSCS();
         getOneDoFJointStateFromSCS();
         multiBodySystem.getRootBody().updateFramesRecursively();
         getExternalWrenchesFromSCS();

         calculator.compute();
         calculator.writeComputedJointAccelerations(multiBodySystem.getJointsToConsider());

         if (performAssertions)
            compareJointAccelerations(1.0e-5 * Math.max(1.0, findAccelerationGreatestMagnitude()));
         compareJointAccelerations(1.0e-3);
      }

      private void getFloatingJointStateFromSCS()
      {
         FloatingJoint scsJoint = (FloatingJoint) robot.getRootJoints().get(0);
         RigidBodyTransform jointTransform3D = scsJoint.getJointTransform3D();
         floatingJoint.getJointPose().set(jointTransform3D);
         floatingJoint.getFrameAfterJoint().update();

         FrameVector3D linearVelocity = new FrameVector3D();
         scsJoint.getVelocity(linearVelocity);
         linearVelocity.changeFrame(floatingJoint.getFrameAfterJoint());
         floatingJoint.getJointTwist().set(scsJoint.getAngularVelocityInBody(), linearVelocity);
      }

      private void getOneDoFJointStateFromSCS()
      {
         for (OneDoFJointBasics joint : allOneDoFJoints)
         {
            PinJoint scsJoint = (PinJoint) robot.getJoint(joint.getName());
            joint.setQ(scsJoint.getQ());
            joint.setQd(scsJoint.getQD());
            double tau = scsJoint.getTau();
            if (scsJoint.tauDamping != null)
               tau += scsJoint.tauDamping.getValue();
            if (scsJoint.tauJointLimit != null)
               tau += scsJoint.tauJointLimit.getValue();
            if (scsJoint.tauVelocityLimit != null)
               tau += scsJoint.tauVelocityLimit.getValue();
            joint.setTau(tau);
         }
      }

      private void getExternalWrenchesFromSCS()
      {
         calculator.setExternalWrenchesToZero();

         for (ExternalForcePoint efp : robot.getAllGroundContactPoints())
         {
            String parentJointName = efp.getParentJoint().getName();
            RigidBodyBasics body = nameToJointMap.get(parentJointName).getSuccessor();

            FrameVector3DReadOnly moment = efp.getYoMoment();
            FrameVector3DReadOnly force = efp.getYoForce();

            FramePoint3D pointOfApplication = new FramePoint3D(efp.getYoPosition());
            pointOfApplication.changeFrame(body.getBodyFixedFrame());

            SpatialVector vector6D = new SpatialVector(moment, force);
            vector6D.changeFrame(body.getBodyFixedFrame());
            Wrench externalWrench = new Wrench(body.getBodyFixedFrame(), body.getBodyFixedFrame());
            externalWrench.set(vector6D.getAngularPart(), vector6D.getLinearPart(), pointOfApplication);

            calculator.getExternalWrench(body).add(externalWrench);
         }
      }

      private static final String FORMAT = EuclidCoreIOTools.getStringFormat(15, 9);

      private void compareJointAccelerations(double epsilon)
      {
         boolean areEqual = true;
         String errorMessage = "Joint accelerations are not equal\n";
         String jointTriggerErrorMessage = "";

         FloatingJoint scsFloatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
         SpatialAcceleration expected = extractFromFloatingJoint(scsFloatingJoint, floatingJoint.getFrameBeforeJoint(), floatingJoint.getFrameAfterJoint());

         if (!floatingJoint.getJointAcceleration().epsilonEquals(expected, epsilon))
         {
            areEqual = false;
            jointTriggerErrorMessage = " floating-joint";
         }

         errorMessage += "Floating joint:\nExpected: " + expected + "\nActual  : " + floatingJoint.getJointAcceleration();

         for (OneDoFJointBasics joint : allOneDoFJoints)
         {
            String jointName = joint.getName();
            OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) robot.getJoint(jointName);
            if (!EuclidCoreTools.epsilonEquals(scsJoint.getQDD(), joint.getQdd(), epsilon))
            {
               areEqual = false;
               jointTriggerErrorMessage += " " + jointName;
            }
            errorMessage += "\n" + jointName + ",\texpected: " + String.format(FORMAT, scsJoint.getQDD()) + ",\tactual: "
                  + String.format(FORMAT, joint.getQdd()) + ", difference: " + String.format(FORMAT, Math.abs(scsJoint.getQDD() - joint.getQdd()));
         }

         if (!areEqual)
         {
            throw new RuntimeException(errorMessage + "\nJoint(s) triggering error:" + jointTriggerErrorMessage);
         }
      }

      private double findAccelerationGreatestMagnitude()
      {
         FloatingJoint scsFloatingJoint = (FloatingJoint) robot.getRootJoints().get(0);
         double mag = scsFloatingJoint.getAngularAccelerationInBody().length();
         Vector3D linearAcceleration = new Vector3D();
         scsFloatingJoint.getLinearAccelerationInWorld(linearAcceleration);
         mag = Math.max(linearAcceleration.length(), mag);

         ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>();
         robot.getAllOneDegreeOfFreedomJoints(joints);
         for (OneDegreeOfFreedomJoint joint : joints)
            mag = Math.max(mag, Math.abs(joint.getQDD()));
         return mag;
      }
   }

   public static SpatialAcceleration extractFromFloatingJoint(FloatingJoint floatingJoint, ReferenceFrame frameBeforeJoint, ReferenceFrame frameAfterJoint)
   {
      ReferenceFrame elevatorFrame = frameBeforeJoint;
      ReferenceFrame bodyFrame = frameAfterJoint;

      FrameVector3D angularVelocityFrameVector = new FrameVector3D();
      FrameVector3D linearVelocityFrameVector = new FrameVector3D();

      floatingJoint.getVelocity(linearVelocityFrameVector);
      linearVelocityFrameVector.changeFrame(bodyFrame);
      floatingJoint.getAngularVelocity(angularVelocityFrameVector, bodyFrame);

      Twist bodyTwist = new Twist(bodyFrame, elevatorFrame, bodyFrame, angularVelocityFrameVector, linearVelocityFrameVector);

      FrameVector3D originAcceleration = new FrameVector3D(elevatorFrame);
      FrameVector3D angularAcceleration = new FrameVector3D(bodyFrame);

      floatingJoint.getLinearAccelerationInWorld(originAcceleration);
      floatingJoint.getAngularAccelerationInBody(angularAcceleration);
      originAcceleration.changeFrame(elevatorFrame);

      SpatialAcceleration spatialAcceleration = new SpatialAcceleration(bodyFrame, elevatorFrame, bodyFrame);
      spatialAcceleration.setBasedOnOriginAcceleration(angularAcceleration, originAcceleration, bodyTwist);
      return spatialAcceleration;
   }
}
