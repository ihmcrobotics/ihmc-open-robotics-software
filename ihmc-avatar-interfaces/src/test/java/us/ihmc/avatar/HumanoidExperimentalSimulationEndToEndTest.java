package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.function.ToDoubleFunction;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

// FIXME Need to improve impulse resolution and minimize tolerances in these tests.
public abstract class HumanoidExperimentalSimulationEndToEndTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected SCS2AvatarTestingSimulation simulationTestHelper = null;

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }
   }

   public void testStanding(TestInfo testInfo) throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             testEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      assertTrue(simulationTestHelper.simulateNow(3.0));
   }

   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             testEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      // Switch to zero-torque controller.
      simulationTestHelper.getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum().set(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      assertTrue(simulationTestHelper.simulateNow(3.0));

      RigidBodyBasics rootBody = simulationTestHelper.getRobot().getRootBody();
      
      assertRigidBodiesAreAboveFlatGround(rootBody,
                                          p -> testEnvironment.getTerrainObject3D().getHeightMapIfAvailable().heightAt(p.getX(), p.getY(), p.getZ()),
                                          3.0e-3);
      assertOneDoFJointsAreWithingLimits(rootBody, 2.0e-2);
   }

   public static void assertRigidBodiesAreAboveFlatGround(RigidBodyBasics rootBody, ToDoubleFunction<Point3DReadOnly> groundHeightFunction, double epsilon)
   {
      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
      {
         if (rigidBody.isRootBody())
            continue;

         FramePoint3D bodyCoM = new FramePoint3D(rigidBody.getBodyFixedFrame());
         bodyCoM.changeFrame(rigidBody.getBodyFixedFrame().getRootFrame());
         double groundHeight = groundHeightFunction.applyAsDouble(bodyCoM);
         assertTrue(bodyCoM.getZ() > groundHeight - epsilon,
                    "Rigid-body is below the ground: " + rigidBody.getName() + ", CoM: " + bodyCoM + ", groundHeight: " + groundHeight);
      }
   }

   public static void assertOneDoFJointsAreWithingLimits(RigidBodyBasics rootBody, double epsilon)
   {
      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) joint;

            double q = oneDoFJoint.getQ();
            double max = oneDoFJoint.getJointLimitUpper();
            double min = oneDoFJoint.getJointLimitLower();

            if (!MathTools.intervalContains(q, min - epsilon, max + epsilon))
               fail("Joint outside limits: " + oneDoFJoint.getName() + ", q = " + q + ", limits = [" + min + ", " + max + "].");
         }
      }
   }
}
