package us.ihmc.quadrupedRobotics.controller.force;

import org.junit.After;
import org.junit.Before;
import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedRobotics.*;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class Quadruped2018PIDemoTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;
   private QuadrupedTestFactory quadrupedTestFactory;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         quadrupedTestFactory.setUsePushRobotController(true);

         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         pusher = new PushRobotTestConductor(conductor.getScs(), "body");
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      quadrupedTestFactory = null;
      conductor = null;
      variables = null;
      pusher = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   public void testFlatGroundWalking(double endPhaseShift, double walkingSpeed)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      double initialDoubleSupportDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();
      double singleSupportDuration = 0.3;

      // this was sending a CoMPositionPacket and wasn't being used in the controller
//      stepTeleopManager.setDesiredBodyHeight(0.8);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.15);
      stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.05);

      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.5 * walkingSpeed, 0.0, 0.0);

      double initialWalkTime = initialDoubleSupportDuration + singleSupportDuration;
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), initialWalkTime));

      double walkTime = 5.0;
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.01);
      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));



      double finalPositionX = walkTime * walkingSpeed * 0.7;
      if(walkingSpeed > 0.0)
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), finalPositionX));
      }
      else
      {
         conductor.addTerminalGoal(YoVariableTestGoal.doubleLessThan(variables.getRobotBodyX(), finalPositionX));
      }

      conductor.simulate();
   }

   public void testWalkingWithPush(double endPhaseShift, double walkingSpeed)
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(endPhaseShift);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.2);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.001);


      stepTeleopManager.setDesiredVelocity(walkingSpeed, 0.0, 0.0);
      stepTeleopManager.requestXGait();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.simulate();

      double duration = 0.2;
      double magnitude = 225;
      pusher.applyForce(new Vector3D(0.0, -1.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.75);
      conductor.simulate();

      pusher.applyForce(new Vector3D(0.0, 1.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.75);
      conductor.simulate();

      duration = 0.5;
      magnitude = 500.0;
      pusher.applyForce(new Vector3D(-1.0, 0.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 2.5);
      conductor.simulate();

      duration = 0.4;
      magnitude = 400.0;
      pusher.applyForce(new Vector3D(1.0, 0.0, 0.0), magnitude, duration);
      PrintTools.info("CoM velocity change = " + magnitude * duration / quadrupedTestFactory.getFullRobotModel().getTotalMass());

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 5.0);
      conductor.simulate();
   }

   public void testMultiGait()
   {
      stepTeleopManager.getXGaitSettings().setStanceWidth(0.25);
      stepTeleopManager.getXGaitSettings().setStanceLength(0.7);
      stepTeleopManager.setDesiredCoMHeight(0.8);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(90);
      stepTeleopManager.getXGaitSettings().setStepDuration(0.3);

      double startingStanceDuration = stepTeleopManager.getXGaitSettings().getEndDoubleSupportDuration();

      stepTeleopManager.requestXGait();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.5);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.7, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 3.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.6, 0.2, 0.4);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 3.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.7 , 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.25);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.001);

      stepTeleopManager.setDesiredVelocity(1.0 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 4.0);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(startingStanceDuration);
      stepTeleopManager.setDesiredVelocity(0.6 , 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.6, -0.2, -0.2);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 6.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.8, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.3);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(startingStanceDuration);


      stepTeleopManager.setDesiredVelocity(0.4 , 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(90);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.25);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.001);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.5 , 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 5.0);
      conductor.simulate();

      stepTeleopManager.getXGaitSettings().setStepDuration(0.3);
      stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(startingStanceDuration);

      stepTeleopManager.setDesiredVelocity(0.5, 0.0, 0.0);
      stepTeleopManager.getXGaitSettings().setEndPhaseShift(90);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();

      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 1.0);
      conductor.simulate();

      stepTeleopManager.requestStanding();

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addDurationGoal(variables.getYoTime(), 0.5);
      conductor.simulate();
   }
}
