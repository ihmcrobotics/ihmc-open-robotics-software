package us.ihmc.avatar;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.robotics.physics.*;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.variable.YoVariable;

public abstract class HumanoidRagdollTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   @BeforeAll
   public static void disableStackTrace()
   {
      YoVariable.SAVE_STACK_TRACE = false;
   }

   public abstract RobotCollisionModel getRobotCollisionModel(CollidableHelper helper, String robotCollisionMask, String... environmentCollisionMasks);

   public void testStanding(TestInfo testInfo) throws Exception
   {
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      DRCSimulationTestHelper drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, testEnvironment);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);

      ThreadTools.sleepForever();
   }

   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      DRCSimulationTestHelper drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, testEnvironment);

      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      // Switch to zero-torque controller.
      drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum()
                             .set(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      HumanoidFloatingRootJointRobot scsRobot = drcSimulationTestHelper.getRobot();
      Vector3D gravity = new Vector3D(scsRobot.getGravityX(), scsRobot.getGravityY(), scsRobot.getGravityZ());

      CollidableHelper helper = new CollidableHelper();
      String environmentCollisionMask = "ground";
      String robotCollisionMask = "robot";
      RobotCollisionModel robotCollisionModel = getRobotCollisionModel(helper, robotCollisionMask, environmentCollisionMask);

      PhysicsEngine customPhysicsEngine = setupCustomPhysicsEngine(drcSimulationTestHelper, robotCollisionModel);
      customPhysicsEngine.addEnvironmentCollidables(toCollidables(helper.getCollisionMask(environmentCollisionMask),
                                                                  helper.createCollisionGroup(robotCollisionMask),
                                                                  testEnvironment));

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      scs.setDT(scs.getDT(), 1);
      simulate(2.5, customPhysicsEngine, scs, gravity);

      ThreadTools.sleepForever();
   }

   private void simulate(double duration, PhysicsEngine physicsEngine, SimulationConstructionSet scs, Vector3DReadOnly gravity)
   {
      double dt = scs.getDT();
      double initialTime = scs.getTime();

      while ((scs.getTime() - initialTime) < duration)
      {
         Stream.of(scs.getRobots()).forEach(Robot::doControllers);
         physicsEngine.simulate(dt, gravity);
         Stream.of(scs.getRobots()).forEach(robot -> robot.getYoTime().add(dt));
         scs.tickAndUpdate();
      }
   }

   private PhysicsEngine setupCustomPhysicsEngine(DRCSimulationTestHelper drcSimulationTestHelper, RobotCollisionModel robotCollisionModel)
   {
      HumanoidFloatingRootJointRobot scsRobot = drcSimulationTestHelper.getRobot();

      PhysicsEngine physicsEngine = new PhysicsEngine();
      YoGraphicsListRegistry yoGraphicsListRegistry = physicsEngine.getPhysicsEngineGraphicsRegistry();
      drcSimulationTestHelper.getYoVariableRegistry().addChild(physicsEngine.getPhysicsEngineRegistry());

      String robotName = getSimpleRobotName();
      RigidBodyBasics rootBody = getRobotModel().createFullRobotModel().getElevator();
      MultiBodySystemStateWriter controllerOutputWriter = createControllerOutputWriter(scsRobot);
      MultiBodySystemStateWriter robotInitialStateWriter = createRobotInitialStateWriter(getRobotModel().getJointMap());
      MultiBodySystemStateReader physicsOutputWriter = createPhysicsOutputWriter(scsRobot);
      physicsEngine.addRobot(robotName, rootBody, controllerOutputWriter, robotInitialStateWriter, robotCollisionModel, physicsOutputWriter);

      drcSimulationTestHelper.getSimulationConstructionSet().addYoGraphicsListRegistry(yoGraphicsListRegistry);
      return physicsEngine;
   }

   private static List<Collidable> toCollidables(int collisionMask, int collisionGroup, CommonAvatarEnvironmentInterface environment)
   {
      return toCollidables(collisionMask, collisionGroup, environment.getTerrainObject3D());
   }

   private static List<Collidable> toCollidables(int collisionMask, int collisionGroup, TerrainObject3D terrainObject3D)
   {
      List<Collidable> collidables = new ArrayList<>();

      for (Shape3DReadOnly terrainShape : terrainObject3D.getTerrainCollisionShapes())
      {
         collidables.add(new Collidable(null, collisionMask, collisionGroup, terrainShape, ReferenceFrame.getWorldFrame()));
      }

      return collidables;
   }

   private MultiBodySystemStateReader createPhysicsOutputWriter(HumanoidFloatingRootJointRobot scsRobot)
   {
      return new MultiBodySystemStateReader()
      {
         private final List<Runnable> stateCopiers = new ArrayList<>();

         @Override
         public void read()
         {
            stateCopiers.forEach(Runnable::run);
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemReadOnly multiBodySystem)
         {
            for (JointReadOnly joint : multiBodySystem.getAllJoints())
            {
               if (joint instanceof OneDoFJointReadOnly)
               {
                  OneDoFJointBasics idJoint = (OneDoFJointBasics) joint;
                  OneDegreeOfFreedomJoint scsJoint = scsRobot.getOneDegreeOfFreedomJoint(idJoint.getName());

                  stateCopiers.add(new Runnable()
                  {
                     @Override
                     public void run()
                     {
                        scsJoint.setQ(idJoint.getQ());
                        scsJoint.setQd(idJoint.getQd());
                        scsJoint.setQdd(idJoint.getQdd());
                        scsJoint.setTau(idJoint.getTau());
                     }
                  });
               }
               else if (joint instanceof FloatingJointReadOnly)
               {
                  FloatingJointReadOnly idJoint = (FloatingJointReadOnly) joint;
                  FloatingJoint scsJoint = scsRobot.getRootJoint();

                  stateCopiers.add(new Runnable()
                  {
                     @Override
                     public void run()
                     {
                        scsJoint.setPosition(idJoint.getJointPose().getPosition());
                        scsJoint.setQuaternion(idJoint.getJointPose().getOrientation());
                        scsJoint.setAngularVelocityInBody(idJoint.getJointTwist().getAngularPart());
                        FrameVector3D linearVelocity = new FrameVector3D(idJoint.getJointTwist().getLinearPart());
                        linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
                        scsJoint.setVelocity(linearVelocity);
                        scsJoint.setAngularAccelerationInBody(idJoint.getJointAcceleration().getAngularPart());
                        FrameVector3D linearAcceleration = new FrameVector3D(idJoint.getJointAcceleration().getLinearPart());
                        linearAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
                        scsJoint.setAcceleration(linearAcceleration);
                     }
                  });
               }
            }
         }
      };
   }

   private MultiBodySystemStateWriter createRobotInitialStateWriter(DRCRobotJointMap jointMap)
   {
      return new MultiBodySystemStateWriter()
      {
         private FloatingJointBasics rootJoint;

         @Override
         public void write()
         {
            rootJoint.getJointPose().setPosition(0.0, 0.0, 1.5);
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            rootJoint = multiBodySystem.getAllJoints().stream().filter(FloatingJointBasics.class::isInstance).map(FloatingJointBasics.class::cast).findFirst()
                                       .get();
         }
      };
   }

   private MultiBodySystemStateWriter createControllerOutputWriter(HumanoidFloatingRootJointRobot scsRobot)
   {
      return new MultiBodySystemStateWriter()
      {
         private final List<Runnable> stateCopiers = new ArrayList<>();

         @Override
         public void write()
         {
            stateCopiers.forEach(Runnable::run);
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            for (JointBasics joint : multiBodySystem.getJointsToConsider())
            {
               if (joint instanceof OneDoFJointBasics)
               {
                  OneDoFJointBasics idJoint = (OneDoFJointBasics) joint;
                  OneDegreeOfFreedomJoint scsJoint = scsRobot.getOneDegreeOfFreedomJoint(joint.getName());
                  stateCopiers.add(() -> idJoint.setTau(scsJoint.getTau()));
               }
            }
         }
      };
   }
}
