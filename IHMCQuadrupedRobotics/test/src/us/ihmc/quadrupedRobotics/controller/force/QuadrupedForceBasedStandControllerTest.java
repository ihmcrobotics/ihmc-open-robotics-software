package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import us.ihmc.euclid.tuple3D.Vector3D;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import junit.framework.AssertionFailedError;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.robotics.partNames.QuadrupedJointName;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

public abstract class QuadrupedForceBasedStandControllerTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      ParameterRegistry.destroyAndRecreateInstance();
   }
   
   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;
      pusher = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingAndResistingPushesOnFrontRightHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.FRONT_RIGHT_HIP_ROLL.getUnderBarName());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingAndResistingPushesOnHindLeftHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_LEFT_HIP_ROLL.getUnderBarName());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingAndResistingPushesOnHindRightHipRoll() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      pushOnShoulder(quadrupedTestFactory, QuadrupedJointName.HIND_RIGHT_HIP_ROLL.getUnderBarName());
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
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
      
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
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
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
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
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingAndResistingPushesOnBody() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      quadrupedTestFactory.setUsePushRobotController(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      pusher = new PushRobotTestConductor(conductor.getScs(), "body");
      
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
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
      
      conductor.concludeTesting();
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 10.0)
   @Test(timeout = 50000)
   public void testStandingUpAndAdjustingCoM() throws IOException
   {
      QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      double stanceHeight = variables.getStanceHeight().getDoubleValue();
      
      variables.getYoComPositionInputZ().set(stanceHeight + 0.05);
      variables.getYoComPositionInputX().set(0.05);
      variables.getYoComPositionInputY().set(-0.05);
      variables.getYoBodyOrientationInputYaw().set(0.05);
      variables.getYoBodyOrientationInputPitch().set(0.05);
      variables.getYoBodyOrientationInputRoll().set(0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight + 0.0);
      variables.getYoComPositionInputX().set(-0.05);
      variables.getYoComPositionInputY().set(-0.05);
      variables.getYoBodyOrientationInputYaw().set(0.05);
      variables.getYoBodyOrientationInputPitch().set(-0.05);
      variables.getYoBodyOrientationInputRoll().set(0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight - 0.05);
      variables.getYoComPositionInputX().set(-0.05);
      variables.getYoComPositionInputY().set(0.05);
      variables.getYoBodyOrientationInputYaw().set(-0.05);
      variables.getYoBodyOrientationInputPitch().set(-0.05);
      variables.getYoBodyOrientationInputRoll().set(0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight - 0.1);
      variables.getYoComPositionInputX().set(0.05);
      variables.getYoComPositionInputY().set(0.05);
      variables.getYoBodyOrientationInputYaw().set(-0.05);
      variables.getYoBodyOrientationInputPitch().set(-0.05);
      variables.getYoBodyOrientationInputRoll().set(-0.05);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      variables.getYoComPositionInputZ().set(stanceHeight);
      variables.getYoComPositionInputX().set(0.0);
      variables.getYoComPositionInputY().set(0.0);
      variables.getYoBodyOrientationInputYaw().set(0.0);
      variables.getYoBodyOrientationInputPitch().set(0.0);
      variables.getYoBodyOrientationInputRoll().set(0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getYoTime(), variables.getYoTime().getDoubleValue() + 1.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
