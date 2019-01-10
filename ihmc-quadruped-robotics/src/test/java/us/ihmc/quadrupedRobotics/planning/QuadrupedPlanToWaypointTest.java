package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedPlanToWaypointTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @Before
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   private void setUpSimulation(TerrainObject3D terrainObject3D)
   {
      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         if(terrainObject3D != null)
         {
            quadrupedTestFactory.setTerrainObject3D(terrainObject3D);
         }

         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 120)
   @Test(timeout = 200000)
   public void testSimpleForwardPoint()
   {
      setUpSimulation(null);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.setEndPhaseShift(180);

      QuadrupedFootstepPlanningRequestPacket planningRequestPacket = new QuadrupedFootstepPlanningRequestPacket();
      planningRequestPacket.getBodyPositionInWorld().set(variables.getRobotBodyX().getDoubleValue(), variables.getRobotBodyY().getDoubleValue(),
                                                         variables.getRobotBodyZ().getDoubleValue());
      planningRequestPacket.getBodyOrientationInWorld().setYawPitchRoll(variables.getBodyEstimateYaw(), variables.getBodyEstimatePitch(),
                                                                        variables.getBodyEstimateRoll());

      planningRequestPacket.getGoalPositionInWorld().set(1.5, 0.5, 0.0);
      planningRequestPacket.getGoalOrientationInWorld().setToYawQuaternion(-Math.PI * 0.25);

      stepTeleopManager.publishPlanningRequest(planningRequestPacket);

      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 1.5, 0.05));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), 0.5, 0.05));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI * 0.25, 0.25));
      conductor.addTimeLimit(variables.getYoTime(), 20.0);
      conductor.addDurationGoal(variables.getYoTime(), 20.0);
      conductor.simulate();
   }

}
