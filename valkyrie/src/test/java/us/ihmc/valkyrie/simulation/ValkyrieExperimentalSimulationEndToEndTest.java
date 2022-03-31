package us.ihmc.valkyrie.simulation;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidExperimentalSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.HumanoidRobotInitialSetup;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieExperimentalSimulationEndToEndTest extends HumanoidExperimentalSimulationEndToEndTest
{
   @Override
   public ValkyrieRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS);
   }

   @Test
   @Override
   public void testStanding(TestInfo testInfo) throws Exception
   {
      super.testStanding(testInfo);
   }

   @Test
   @Override
   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      super.testZeroTorque(testInfo);
   }

   @Test
   public void testFlyingZeroTorque(TestInfo testInfo) throws Exception
   {
      simulationTestingParameters.setUsePefectSensors(true);

      ValkyrieRobotModel robotModel = getRobotModel();
      robotModel.setRobotInitialSetup(new FlyingValkyrieInitialSetup(robotModel.getJointMap()));
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             testEnvironment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      // Switch to zero-torque controller.
      simulationTestHelper.getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum().set(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      assertTrue(simulationTestHelper.simulateAndWait(3.0));

      RigidBodyBasics elevator = simulationTestHelper.getControllerFullRobotModel().getElevator();
      assertRigidBodiesAreAboveFlatGround(elevator,
                                          p -> testEnvironment.getTerrainObject3D().getHeightMapIfAvailable().heightAt(p.getX(), p.getY(), p.getZ()),
                                          0.0);
      assertOneDoFJointsAreWithingLimits(elevator, 2.0e-2);
   }

   private static class FlyingValkyrieInitialSetup extends HumanoidRobotInitialSetup
   {
      public FlyingValkyrieInitialSetup(HumanoidJointNameMap jointMap)
      {
         super(jointMap);

         { // LEFT side
            setJoint(RobotSide.LEFT, LegJointName.HIP_PITCH, -0.7);
            setJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH, 1.5);
            setJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH, 0.7);

            setJoint(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH, 1.1);
            setJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL, -1.2);
            setJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH, -2.0);
         }

         { // RIGHT side
            setJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH, 0.3);
            setJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH, 0.0);
            setJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH, 0.8);

            setJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_PITCH, 0.0);
            setJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL, -1.3);
            setJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_YAW, -1.5);
            setJoint(RobotSide.RIGHT, ArmJointName.ELBOW_PITCH, 0.3);
         }

         rootJointOrientation.setYawPitchRoll(0.0, 1.0, 0.0);
         rootJointPosition.set(-20.0, 0.0, 1.0);
         rootJointAngularVelocityInBody.set(0.0, 1.0, 0.0);
         rootJointLinearVelocityInWorld.set(20.0, 0.0, 8.0);
      }
   }
}
