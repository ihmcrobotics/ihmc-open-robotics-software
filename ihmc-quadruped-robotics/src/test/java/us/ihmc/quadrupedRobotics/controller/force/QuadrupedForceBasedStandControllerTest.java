package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import us.ihmc.euclid.tuple3D.Vector3D;

import org.junit.After;
import org.junit.Before;

import junit.framework.AssertionFailedError;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import static junit.framework.TestCase.assertTrue;

public abstract class QuadrupedForceBasedStandControllerTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      pusher = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_RIGHT_HIP_ROLL.getUnderBarName());
   }
   
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_LEFT_HIP_ROLL.getUnderBarName());
   }
   
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_RIGHT_HIP_ROLL.getUnderBarName());
   }
   
   public void testStandingAndResistingPushesOnFrontLeftHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_LEFT_HIP_ROLL.getUnderBarName());
   }

   private void pushOnShoulder(QuadrupedTestFactory quadrupedTestFactory, String jointToPushOn) throws IOException, AssertionFailedError
   {
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      pusher = new PushRobotTestConductor(conductor.getScs(), jointToPushOn);

      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), 30.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(-1.0, -1.0, 0.0), 50.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(1.0, -1.0, 0.0), 50.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(1.0, 1.0, 0.0), 50.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(-1.0, 1.0, 0.0), 50.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(0.0, 0.0, 1.0), 50.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(0.0, 0.0, -1.0), 50.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(0.0, 0.0, 1.0), 200.0, 1.0);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
   }

   public void testStandingAndResistingHumanPowerKickToFace() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      pusher = new PushRobotTestConductor(conductor.getScs(), "head_roll");
      
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 0.5));
      conductor.simulate();
      
      pusher.applyForce(new Vector3D(-1.0, -0.1, 0.75), 700.0, 0.05);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 2.0));
      conductor.simulate();
   }
   
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      pusher = new PushRobotTestConductor(conductor.getScs(), "body");
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
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
   
   public void testStandingUpAndAdjustingCoM(double translationShift, double translationDelta, double orientationShift, double orientationDelta) throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      double initialComZ = variables.getComPositionEstimateZ().getDoubleValue();
      testMovingCoM(translationShift, -translationShift, initialComZ + translationShift, orientationShift, orientationShift, orientationShift, translationDelta, orientationDelta);
      testMovingCoM(-translationShift, -translationShift, initialComZ, orientationShift, - orientationShift, orientationShift, translationDelta, orientationDelta);
      testMovingCoM(-translationShift,  translationShift, initialComZ - translationShift, - orientationShift, - orientationShift, - orientationShift, translationDelta, orientationDelta);
      testMovingCoM(translationShift, translationShift, initialComZ + translationShift, - orientationShift, -orientationShift, -orientationShift, translationDelta, orientationDelta);
      testMovingCoM(0.0, 0.0, initialComZ, 0.0, 0.0, 0.0, translationDelta, orientationDelta);
   }

   private void testMovingCoM(double comPositionX, double comPositionY, double comPositionZ, double bodyOrientationYaw, double bodyOrientationPitch, double bodyOrientationRoll, double translationDelta, double orientationDelta)
   {
      variables.getYoComPositionInputX().set(comPositionX);
      variables.getYoComPositionInputY().set(comPositionY);
      variables.getYoComPositionInputZ().set(comPositionZ);
      variables.getYoBodyOrientationInputYaw().set(bodyOrientationYaw);
      variables.getYoBodyOrientationInputPitch().set(bodyOrientationPitch);
      variables.getYoBodyOrientationInputRoll().set(bodyOrientationRoll);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();

      assertTrue(Math.abs(variables.getComPositionEstimateX().getDoubleValue() - variables.getComPositionSetpointX().getDoubleValue()) < translationDelta);
      assertTrue(Math.abs(variables.getComPositionEstimateY().getDoubleValue() - variables.getComPositionSetpointY().getDoubleValue()) < translationDelta);
      assertTrue(Math.abs(variables.getComPositionEstimateZ().getDoubleValue() - variables.getComPositionSetpointZ().getDoubleValue()) < translationDelta);
      assertTrue(Math.abs(variables.getComPositionEstimateYaw().getDoubleValue() - bodyOrientationYaw) < orientationDelta);
      assertTrue(Math.abs(variables.getComPositionEstimatePitch().getDoubleValue() - bodyOrientationPitch) < orientationDelta);
      assertTrue(Math.abs(variables.getComPositionEstimateRoll().getDoubleValue() - bodyOrientationRoll) < orientationDelta);
   }
}
