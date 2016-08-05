package us.ihmc.quadrupedRobotics.controller.force.speedTorqueLimits;

import java.io.IOException;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;

public abstract class QuadrupedSpeedTorqueLimitGraphing implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;
   
   public void createSimulation() throws IOException
   {
      QuadrupedTestFactory testFactory = createQuadrupedTestFactory();
      testFactory.setControlMode(QuadrupedControlMode.FORCE);
      testFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      testFactory.setUsePushRobotController(true);
      testFactory.setUseStateEstimator(true);
      conductor = testFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      pusher = new PushRobotTestConductor(conductor.getScs(), "body");
   }
   
   public void pushDownOnBodyUntilLimits()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      
      double force = 0.0;
      double forceIncreasePerSec = 200.0;
      double dt = 0.1;
      
      while (true)
      {
         try
         {
            pusher.applyForce(new Vector3d(0.0, 0.0, -1.0), force, dt);
            
            conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
            conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, dt));
            conductor.simulate();
            
            force += forceIncreasePerSec * dt;
         }
         catch (AssertionError assertionError)
         {
            break;
         }
      }
   }
}
