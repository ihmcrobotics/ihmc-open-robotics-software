package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.jupiter.api.*;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;

@Disabled
@Tag("quadruped-xgait-2")
public abstract class QuadrupedXGaitFlyingTrotTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   public abstract double getFastWalkingSpeed();
   public abstract double getSlowWalkingSpeed();
   public abstract double getWalkingAngularVelocity();
   public abstract double getWalkingSpeedWhileTurning();

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }


   @Disabled
   @Test
   public void testFlyingTrotInPlace()
   {
      testFlyingTrot(0.0, 0.0, 0.0);
   }

   @Disabled
   @Test
   public void testFlyingTrotForwardSlow()
   {
      testFlyingTrot(getSlowWalkingSpeed(), 0.0, 0.0);
   }

   @Disabled
   @Test
   public void testFlyingTrotForwardFast()
   {
      testFlyingTrot(getFastWalkingSpeed(), 0.0, 0.0);
   }

   @Test
   public void testFlyingTrotLeftSlow()
   {
      testFlyingTrot(0.0, getFastWalkingSpeed(), 0.0);
   }

   @Test
   public void testFlyingTrotRightSlow()
   {
      testFlyingTrot(0.0, -getSlowWalkingSpeed(), 0.0);
   }

   @Test
   public void testFlyingTrotTurnLeftInPlace()
   {
      testFlyingTrot(0.0, 0.0, -getWalkingAngularVelocity());
   }

   @Test
   public void testFlyingTrotTurnRightInPlace()
   {
      testFlyingTrot(0.0, 0.0, getWalkingAngularVelocity());
   }

   @Test
   public void testFlyingTrotTurnInAForwardLeftCircle()
   {
      testFlyingTrot(getWalkingSpeedWhileTurning(), 0.0, -getWalkingAngularVelocity());
   }

   @Test
   public void testFlyingTrotTurnInAForwardRightCircle()
   {
      testFlyingTrot(getWalkingSpeedWhileTurning(), 0.0, getWalkingAngularVelocity());
   }


   private void setUpVariablesForFlyingTrot()
   {
      YoRegistry rootRegistry = conductor.getScs().getRootRegistry();
      YoBoolean requireTwoFeetInContact = ((YoBoolean) rootRegistry.findVariable("requireTwoFeetInContact"));
      YoBoolean requireFootOnEachEnd = ((YoBoolean) rootRegistry.findVariable("requireFootOnEachEnd"));
      YoDouble rhoClampingDuration = ((YoDouble) rootRegistry.findVariable("rhoClampingDuration"));
      YoDouble loadingMaxMagnitude = ((YoDouble) rootRegistry.findVariable("loadingMaxMagnitude"));
      YoDouble kp_comHeight = ((YoDouble) rootRegistry.findVariable("kp_comHeight"));

      requireFootOnEachEnd.set(false);
      requireTwoFeetInContact.set(false);
      rhoClampingDuration.set(0.01);
      loadingMaxMagnitude.set(1000.0);
      kp_comHeight.set(250.0);
   }
   private void testFlyingTrot(double forwardWalkingSpeed, double sidewaysSpeed, double turningSpeed)
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      setUpVariablesForFlyingTrot();

      stepTeleopManager.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      stepTeleopManager.setStepDuration(QuadrupedSpeed.FAST, QuadrupedGait.TROT.getEndPhaseShift(), 0.33);
      stepTeleopManager.setEndDoubleSupportDuration(QuadrupedSpeed.FAST, QuadrupedGait.TROT.getEndPhaseShift(), -0.1);

      double walkTime = 6.0;
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(forwardWalkingSpeed, sidewaysSpeed, turningSpeed);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));

      double finalPositionX = walkTime * forwardWalkingSpeed * 0.7;
      if(forwardWalkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 1.0));

      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getSteppingState(), QuadrupedSteppingStateEnum.STAND));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 0.5));

      conductor.simulate();

      QuadrupedTestBehaviors.sitDown(conductor, variables);
   }
}
