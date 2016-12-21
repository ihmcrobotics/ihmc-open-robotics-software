package us.ihmc.quadrupedRobotics.controller.force;

import java.io.IOException;

import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.robotics.dataStructures.parameter.ParameterRegistry;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public abstract class QuadrupedXGaitPushRecoveryTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      try
      {
         ParameterRegistry.destroyAndRecreateInstance();
         QuadrupedTestFactory quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUsePushRobotController(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         pusher = new PushRobotTestConductor(conductor.getScs(), "neck_root_yaw");
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }
   
   @After
   public void tearDown()
   {
      conductor = null;
      variables = null;
      pusher = null;
      
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 30000)
   public void testWalkingForwardFastWithPush()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
      variables.getUserTrigger().set(QuadrupedForceControllerRequestedEvent.REQUEST_XGAIT);
      variables.getYoPlanarVelocityInputX().set(0.8);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 1.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3d(0.0, -1.0, 0.0), 50.0, 0.3);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 4.0));
      conductor.simulate();
      
      pusher.applyForce(new Vector3d(0.0, 1.0, 0.0), 50.0, 0.3);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTimeLimit(variables.getYoTime(), 5.0);
      conductor.addTerminalGoal(YoVariableTestGoal.doubleGreaterThan(variables.getRobotBodyX(), 7.0));
      conductor.simulate();
      
      conductor.concludeTesting();
   }
}
