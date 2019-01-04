package us.ihmc.quadrupedRobotics.controller.force;

import junit.framework.AssertionFailedError;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedPlanning.input.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

import static junit.framework.TestCase.assertTrue;

public abstract class QuadrupedForceBasedStandControllerTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private PushRobotTestConductor pusher;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      quadrupedTestFactory.setUseNetworking(true);
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;
      pusher = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public abstract double getHeightShift();
   public abstract double getHeightDelta();
   public abstract double getTranslationShift();
   public abstract double getTranslationDelta();
   public abstract double getOrientationShift();
   public abstract double getOrientationDelta();


   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 320000)
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_RIGHT_HIP_ROLL.getUnderBarName());
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 320000)
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_LEFT_HIP_ROLL.getUnderBarName());
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 320000)
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_RIGHT_HIP_ROLL.getUnderBarName());
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 320000)
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_LEFT_HIP_ROLL.getUnderBarName());
   }


   private void pushOnShoulder(QuadrupedTestFactory quadrupedTestFactory, String jointToPushOn) throws IOException, AssertionFailedError
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      pusher = new PushRobotTestConductor(conductor.getScs(), jointToPushOn);

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), 30.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(-1.0, -1.0, 0.0), 50.0, 0.25);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.25));
      conductor.simulate();

      pusher.applyForce(new Vector3D(1.0, -1.0, 0.0), 50.0, 0.25);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.25));
      conductor.simulate();

      pusher.applyForce(new Vector3D(1.0, 1.0, 0.0), 50.0, 0.25);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.25));
      conductor.simulate();

      pusher.applyForce(new Vector3D(-1.0, 1.0, 0.0), 50.0, 0.25);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.25));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 0.0, 1.0), 50.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 0.0, -1.0), 50.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 0.0, 1.0), 200.0, 0.5);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
   }

   public void testStandingAndResistingHumanPowerKickToFace() throws IOException
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      pusher = new PushRobotTestConductor(conductor.getScs(), "head_roll");

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.5));
      conductor.simulate();

      pusher.applyForce(new Vector3D(-1.0, -0.1, 0.75), 700.0, 0.05);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();
   }

   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 550000)
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      pusher = new PushRobotTestConductor(conductor.getScs(), "body");

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), 30.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(-1.0, -1.0, 0.0), 30.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 0.0, 1.0), 50.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 0.0, -1.0), 50.0, 1.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 0.0, 1.0), 200.0, 0.5);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();
   }

   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 390000)
   public void testStandingUpAndAdjustingCoM()
         throws IOException
   {
      double heightShift = getHeightShift();
      double heightDelta = getHeightDelta();
      double translationShift = getTranslationShift();
      double translationDelta = getTranslationDelta();
      double orientationShift = getOrientationShift();
      double orientationDelta = getOrientationDelta();
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      double initialBodyHeight = variables.getCurrentHeightInWorld().getDoubleValue();
      runMovingCoM(initialBodyHeight + heightShift, orientationShift, orientationShift, orientationShift, translationShift, translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight + heightShift, orientationShift, orientationShift, orientationShift,-translationShift, translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight + heightShift, orientationShift, orientationShift, orientationShift,-translationShift,-translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight, orientationShift, -orientationShift, orientationShift, translationShift, -translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight - heightShift, -orientationShift, -orientationShift, -orientationShift, translationShift, -translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight + heightShift, -orientationShift, -orientationShift, -orientationShift, translationShift, translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight + heightShift, orientationShift, orientationShift, orientationShift, -translationShift, -translationShift, heightDelta, orientationDelta, translationDelta);
      runMovingCoM(initialBodyHeight, 0.0, 0.0, 0.0, 0.0, 0.0, heightDelta, orientationDelta, translationDelta);
   }

   private void runMovingCoM(double bodyHeight, double bodyOrientationYaw, double bodyOrientationPitch, double bodyOrientationRoll, double bodyX,
                             double bodyY, double heightDelta, double orientationDelta, double translationDelta)
   {
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);
      stepTeleopManager.setDesiredBodyPose(bodyX, bodyY, bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll, 0.1);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      assertTrue("Height did not meet goal : Math.abs(" + variables.getCurrentHeightInWorld().getDoubleValue() + " - " + variables.getHeightInWorldSetpoint()
                                                                                                                                  .getDoubleValue() + " < "
                       + heightDelta,
                 Math.abs(variables.getCurrentHeightInWorld().getDoubleValue() - variables.getHeightInWorldSetpoint().getDoubleValue()) < heightDelta);
      assertTrue("Yaw did not meet goal : Math.abs(" + variables.getBodyEstimateYaw() + " - " + bodyOrientationYaw + " < "
                       + orientationDelta, Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(variables.getBodyEstimateYaw(), bodyOrientationYaw)) < orientationDelta);
      assertTrue("Pitch did not meet goal : Math.abs(" + variables.getBodyEstimatePitch() + " - " + bodyOrientationPitch + " < "
                       + orientationDelta,
                 Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(variables.getBodyEstimatePitch(), bodyOrientationPitch)) < orientationDelta);
      assertTrue("Roll did not meet goal : Math.abs(" + variables.getBodyEstimateRoll() + " - " + bodyOrientationRoll + " < "
                       + orientationDelta,
                 Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(variables.getBodyEstimateRoll(), bodyOrientationRoll)) < orientationDelta);
   }
}
