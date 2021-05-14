package us.ihmc.valkyrie.simulation;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.HumanoidExperimentalSimulationEndToEndTest;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

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
      robotModel.setRobotInitialSetup(new FlyingValkyrieInitialSetup());
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, testEnvironment);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      // Switch to zero-torque controller.
      drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum()
                             .set(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      drcSimulationTestHelper.getSimulationConstructionSet().setCameraTracking(true, true, true, true);
      drcSimulationTestHelper.getSimulationConstructionSet().setCameraDolly(true, true, true, true);
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));

      RigidBodyBasics elevator = drcSimulationTestHelper.getControllerFullRobotModel().getElevator();
      assertRigidBodiesAreAboveFlatGround(elevator, p -> testEnvironment.getTerrainObject3D().getHeightMapIfAvailable().heightAt(p.getX(), p.getY(), p.getZ()), 0.0);
      assertOneDoFJointsAreWithingLimits(elevator, 2.0e-2);
   }

   private static class FlyingValkyrieInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
   {

      private HumanoidFloatingRootJointRobot robot;
      private HumanoidJointNameMap jointMap;

      @Override
      public void initializeRobot(HumanoidFloatingRootJointRobot robot, HumanoidJointNameMap jointMap)
      {
         this.robot = robot;
         this.jointMap = jointMap;

         setJoint(RobotSide.LEFT, LegJointName.HIP_PITCH, -0.7, 0.0);
         setJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH, 1.5, 0.0);
         setJoint(RobotSide.LEFT, LegJointName.ANKLE_PITCH, 0.7, 0.0);

         setJoint(RobotSide.RIGHT, LegJointName.HIP_PITCH, 0.3, 0.0);
         setJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH, 0.0, 0.0);
         setJoint(RobotSide.RIGHT, LegJointName.ANKLE_PITCH, 0.8, 0.0);

         setJoint(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH, 1.1, 0.0);
         setJoint(RobotSide.LEFT, ArmJointName.SHOULDER_ROLL, -1.2, 0.0);
         setJoint(RobotSide.LEFT, ArmJointName.ELBOW_PITCH, -2.0, 0.0);

         setJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_PITCH, 0.0, 0.0);
         setJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_ROLL, -1.3, 0.0);
         setJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_YAW, -1.5, 0.0);
         setJoint(RobotSide.RIGHT, ArmJointName.ELBOW_PITCH, 0.3, 0.0);

         setJoint(SpineJointName.SPINE_ROLL, -0.1, 0.0);

         setJoint(NeckJointName.DISTAL_NECK_PITCH, -0.8, 0.0);
         setJoint(NeckJointName.DISTAL_NECK_YAW, -0.1, 0.0);

         robot.getRootJoint().setPosition(-20.0, 0.0, 1.0);
         robot.getRootJoint().setRotation(new RotationMatrix(new YawPitchRoll(0.0, 1.0, 0.0)));

         robot.getRootJoint().setVelocity(8.0, 0.0, 20.0);
         robot.getRootJoint().setAngularVelocityInBody(new Vector3D(0.0, 1.5, 0.0));
      }

      private void setJoint(RobotSide robotSide, LegJointName legJointName, double q, double qd)
      {
         setJoint(jointMap.getLegJointName(robotSide, legJointName), q, qd);
      }

      private void setJoint(RobotSide robotSide, ArmJointName armJointName, double q, double qd)
      {
         setJoint(jointMap.getArmJointName(robotSide, armJointName), q, qd);
      }

      private void setJoint(SpineJointName spineJointName, double q, double qd)
      {
         setJoint(jointMap.getSpineJointName(spineJointName), q, qd);
      }

      private void setJoint(NeckJointName neckJointName, double q, double qd)
      {
         setJoint(jointMap.getNeckJointName(neckJointName), q, qd);
      }

      private void setJoint(String jointName, double q, double qd)
      {
         OneDegreeOfFreedomJoint joint = robot.getOneDegreeOfFreedomJoint(jointName);
         joint.setQ(q);
         joint.setQd(qd);
      }

      @Override
      public void setInitialYaw(double yaw)
      {

      }

      @Override
      public double getInitialYaw()
      {
         return 0;
      }

      @Override
      public void setInitialGroundHeight(double groundHeight)
      {
      }

      @Override
      public double getInitialGroundHeight()
      {
         return 0;
      }

      @Override
      public void setOffset(Vector3D additionalOffset)
      {
      }

      @Override
      public void getOffset(Vector3D offsetToPack)
      {
      }
   }

}
