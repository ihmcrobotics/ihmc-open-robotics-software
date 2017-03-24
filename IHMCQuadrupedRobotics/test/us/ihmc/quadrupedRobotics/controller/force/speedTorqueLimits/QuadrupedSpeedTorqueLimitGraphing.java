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
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionsettools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.tools.thread.ThreadTools;

public abstract class QuadrupedSpeedTorqueLimitGraphing implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private PushRobotTestConductor pusher;
   
   public SimulationConstructionSet createSimulation() throws IOException
   {
      QuadrupedTestFactory testFactory = createQuadrupedTestFactory();
      testFactory.setControlMode(QuadrupedControlMode.FORCE);
      testFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
      testFactory.setUsePushRobotController(true);
      testFactory.setUseStateEstimator(true);
      conductor = testFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      pusher = new PushRobotTestConductor(conductor.getScs(), "body");
      
      return conductor.getScs();
   }
   
   public void trotAroundSuperAggressively()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
      QuadrupedTestBehaviors.enterXGait(conductor, variables);
      
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
            
            variables.getXGaitEndPhaseShiftInput().set(180.0);
            variables.getXGaitStepGroundClearanceInput().set(0.25);
            variables.getXGaitStepDurationInput().set(0.55);
//            variables.getXGaitStanceWidthInput().set(0.3);
            variables.getXGaitEndDoubleSupportDurationInput().set(0.0);
            variables.getYoBodyOrientationInputYaw().set(0.05 * Math.cos(variables.getYoTime().getDoubleValue()));
            variables.getYoBodyOrientationInputPitch().set(0.1 * Math.sin(variables.getYoTime().getDoubleValue()));
            variables.getYoBodyOrientationInputRoll().set(0.05 * Math.cos(variables.getYoTime().getDoubleValue()));
            variables.getYoComPositionInputZ().set(0.03 * Math.sin(variables.getYoTime().getDoubleValue()) + 0.55);
            variables.getYoPlanarVelocityInputX().set(0.2 * Math.sin(variables.getYoTime().getDoubleValue() + Math.PI) + 0.3);
            variables.getYoPlanarVelocityInputY().set(0.2 * Math.cos(variables.getYoTime().getDoubleValue() + Math.PI));
            variables.getYoPlanarVelocityInputZ().set(0.2 * Math.cos(variables.getYoTime().getDoubleValue()));
            
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
      QuadrupedTestBehaviors.readyXGait(conductor, variables);
      
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
