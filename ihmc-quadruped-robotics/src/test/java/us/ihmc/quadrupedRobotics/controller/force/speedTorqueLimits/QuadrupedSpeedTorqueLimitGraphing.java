package us.ihmc.quadrupedRobotics.controller.force.speedTorqueLimits;

import java.io.IOException;

import us.ihmc.euclid.tuple3D.Vector3D;

import us.ihmc.commonWalkingControlModules.pushRecovery.PushRobotTestConductor;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.commons.thread.ThreadTools;

public abstract class QuadrupedSpeedTorqueLimitGraphing implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private PushRobotTestConductor pusher;
   
   public SimulationConstructionSet createSimulation() throws IOException
   {
      QuadrupedTestFactory testFactory = createQuadrupedTestFactory();
      testFactory.setControlMode(QuadrupedControlMode.FORCE);
      testFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      testFactory.setUsePushRobotController(true);
      testFactory.setUseStateEstimator(true);
      testFactory.setUseNetworking(true);
      conductor = testFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = testFactory.getStepTeleopManager();
      pusher = new PushRobotTestConductor(conductor.getScs(), "body");

      return conductor.getScs();
   }
   
   public void trotAroundSuperAggressively()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      double dt = 0.1;
      double phase = 90.0;
      
      while (variables.getYoTime().getDoubleValue() < 20.0)
      {
         try
         {
//            pusher.applyForce(new Vector3d(0.0, 0.0, -1.0), force, dt);
            phase += dt * 20.0;
            
            stepTeleopManager.getXGaitSettings().setEndPhaseShift(180.0);
            stepTeleopManager.getXGaitSettings().setStepGroundClearance(0.25);
            stepTeleopManager.getXGaitSettings().setStepDuration(0.55);
            stepTeleopManager.getXGaitSettings().setEndDoubleSupportDuration(0.0);
            double yaw = 0.05 * Math.cos(variables.getYoTime().getDoubleValue());
            double pitch = 0.1 * Math.sin(variables.getYoTime().getDoubleValue());
            double roll = 0.05 * Math.cos(variables.getYoTime().getDoubleValue());
            stepTeleopManager.setDesiredBodyOrientation(yaw, pitch, roll, 0.0);
            stepTeleopManager.setDesiredCoMHeight(0.03 * Math.sin(variables.getYoTime().getDoubleValue()) + 0.55);
            double planarVelocityInputX = 0.2 * Math.sin(variables.getYoTime().getDoubleValue() + Math.PI) + 0.3;
            double planarVelocityInputY = 0.2 * Math.cos(variables.getYoTime().getDoubleValue() + Math.PI);
            double planarVelocityInputZ = 0.2 * Math.cos(variables.getYoTime().getDoubleValue());
            stepTeleopManager.setDesiredVelocity(planarVelocityInputX, planarVelocityInputY, planarVelocityInputZ);
            
            conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
            conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, dt));
            conductor.simulate();
         }
         catch (AssertionError assertionError)
         {
            break;
         }
      }
      
      ThreadTools.sleepForever();
   }
   
   public void pushDownOnBodyUntilLimits()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      
      double force = 0.0;
      double forceIncreasePerSec = 200.0;
      double dt = 0.1;
      
      while (true)
      {
         try
         {
            pusher.applyForce(new Vector3D(0.0, 0.0, -1.0), force, dt);
            
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
