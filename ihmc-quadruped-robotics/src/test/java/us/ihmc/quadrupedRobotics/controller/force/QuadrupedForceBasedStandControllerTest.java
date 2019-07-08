package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.jupiter.api.*;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

@Tag("quadruped-force-based")
public abstract class QuadrupedForceBasedStandControllerTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private PushRobotTestConductor pusher;
   private QuadrupedTestFactory quadrupedTestFactory;

   private static final double bodyShiftDuration = 0.6;
   private static final double comShiftDuration = 1.0;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
   }

   @AfterEach
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


   @Test
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_RIGHT_HIP_ROLL.getUnderBarName());
   }

   @Test
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_LEFT_HIP_ROLL.getUnderBarName());
   }

   @Test
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_RIGHT_HIP_ROLL.getUnderBarName());
   }

   @Test
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_LEFT_HIP_ROLL.getUnderBarName());
   }


   private void pushOnShoulder(QuadrupedTestFactory quadrupedTestFactory, String jointToPushOn) throws IOException
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
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
      variables = new QuadrupedTestYoVariables(conductor.getScs());
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

   @Test
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
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

   @Test
   public void testStandingUpAndAdjustingBody() throws IOException
   {
      double heightShift = getHeightShift();
      double heightDelta = getHeightDelta();
      double orientationShift = getOrientationShift();
      double orientationDelta = getOrientationDelta();
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.addTimeLimit(variables.getYoTime(), 2.0);
      conductor.simulate();

      double initialBodyHeight = variables.getCurrentHeightInWorld().getDoubleValue();
      runMovingBody(initialBodyHeight - heightShift , orientationShift, orientationShift, orientationShift, heightDelta, orientationDelta);
      runMovingBody(initialBodyHeight - heightShift, orientationShift, -orientationShift, orientationShift, heightDelta, orientationDelta);
      runMovingBody(initialBodyHeight - heightShift, -orientationShift, -orientationShift, -orientationShift, heightDelta, orientationDelta);
      runMovingBody(initialBodyHeight - heightShift, orientationShift, orientationShift, orientationShift, heightDelta, orientationDelta);
      runMovingBody(initialBodyHeight, 0.0, 0.0, 0.0, heightDelta, orientationDelta);
   }

   @Disabled
   @Test
   public void testStandingUpAndShiftingCoM() throws IOException
   {
      double heightShift = getHeightShift();
      double heightDelta = 5.0 * getHeightDelta();
      double translationShift = getTranslationShift();
      double translationDelta = getTranslationDelta();
      quadrupedTestFactory.setInitialOffset(new QuadrupedInitialOffsetAndYaw(1.0));
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();

      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);


      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));
      conductor.addTimeLimit(variables.getYoTime(), 2.0);
      conductor.simulate();

      double initialBodyHeight = variables.getCurrentHeightInWorld().getDoubleValue();
      runMovingCoM(initialBodyHeight + heightShift, translationShift, translationShift, heightDelta, translationDelta);
      runMovingCoM(initialBodyHeight + heightShift,-translationShift, translationShift, heightDelta, translationDelta);
      runMovingCoM(initialBodyHeight + heightShift,-translationShift,-translationShift, heightDelta, translationDelta);
      runMovingCoM(initialBodyHeight, translationShift, -translationShift, heightDelta, translationDelta);
      runMovingCoM(initialBodyHeight, 0.0, 0.0, heightDelta, translationDelta);
   }

   private void runMovingBody(double bodyHeight, double bodyOrientationYaw, double bodyOrientationPitch, double bodyOrientationRoll,  double heightDelta,
                              double orientationDelta)
   {
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);
      stepTeleopManager.setDesiredBodyOrientation(bodyOrientationYaw, bodyOrientationPitch, bodyOrientationRoll, bodyShiftDuration);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 5.0 * bodyShiftDuration));
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

   private void runMovingCoM(double bodyHeight, double bodyX, double bodyY, double heightDelta, double translationDelta)
   {
      stepTeleopManager.setDesiredBodyHeight(bodyHeight);

      QuadrupedReferenceFrames referenceFrames = new QuadrupedReferenceFrames(quadrupedTestFactory.getFullRobotModel());
      referenceFrames.updateFrames();

      FramePoint2D location = new FramePoint2D(referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
      location.set(bodyX, bodyY);
      location.changeFrame(ReferenceFrame.getWorldFrame());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 2.0 * comShiftDuration));
      conductor.addTimeLimit(variables.getYoTime(), 4.0 * comShiftDuration);
      conductor.simulate();

      double currentHeightInWorld = variables.getCurrentHeightInWorld().getDoubleValue();
      double currentBodyXInWorld = variables.getRobotBodyX().getDoubleValue();
      double currentBodyYInWorld = variables.getRobotBodyY().getDoubleValue();

      double heightInWorldSetpoint = variables.getHeightInWorldSetpoint().getDoubleValue();
      double bodyInWorldXSetpoint = location.getX();
      double bodyInWorldYSetpoint = location.getY();

      assertEquals("Height did not meet goal.", heightInWorldSetpoint, currentHeightInWorld, heightDelta);
      assertEquals("X did not meet goal.", bodyInWorldXSetpoint, currentBodyXInWorld, translationDelta);
      assertEquals("Y did not meet goal.", bodyInWorldYSetpoint, currentBodyYInWorld, translationDelta);
   }
}
