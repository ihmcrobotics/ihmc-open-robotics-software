package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

@Tag("quadruped-planning")
public abstract class QuadrupedAStarFootstepSimulationPlanToWaypointTest implements QuadrupedMultiRobotTestInterface
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   private void setUpSimulation(TerrainObject3D terrainObject3D)
   {
      try
      {
         quadrupedTestFactory = createQuadrupedTestFactory();
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

   @AfterEach
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testSimpleForwardPoint()
   {
      setUpSimulation(null);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);
      stepTeleopManager.setEndPhaseShift(180);

      ROS2Tools.createCallbackSubscription(stepTeleopManager.getRos2Node(), QuadrupedFootstepPlanningToolboxOutputStatus.class,
                                           ROS2Tools.getTopicNameGenerator(stepTeleopManager.getRobotName(), ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX,
                                                                           ROS2TopicQualifier.OUTPUT),
                                           s -> stepTeleopManager.publishTimedStepListToController(s.takeNextData().getFootstepDataList()));
      ROS2Tools.createCallbackSubscription(stepTeleopManager.getRos2Node(), QuadrupedBodyOrientationMessage.class,
                                           ROS2Tools.getTopicNameGenerator(stepTeleopManager.getRobotName(), ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX,
                                                                           ROS2TopicQualifier.OUTPUT),
                                           s -> stepTeleopManager.publishBodyOrientationMessage(s.takeNextData()));

      QuadrupedFootstepPlanningRequestPacket planningRequestPacket = new QuadrupedFootstepPlanningRequestPacket();
      planningRequestPacket.getBodyPositionInWorld().set(variables.getRobotBodyX().getDoubleValue(), variables.getRobotBodyY().getDoubleValue(),
                                                         variables.getRobotBodyZ().getDoubleValue());
      planningRequestPacket.getBodyOrientationInWorld().setYawPitchRoll(variables.getBodyEstimateYaw(), variables.getBodyEstimatePitch(),
                                                                        variables.getBodyEstimateRoll());

      planningRequestPacket.getGoalPositionInWorld().set(1.5, 0.5, 0.0);
      planningRequestPacket.getGoalOrientationInWorld().setToYawQuaternion(-Math.PI * 0.25);
      planningRequestPacket.setRequestedFootstepPlannerType(FootstepPlannerType.A_STAR.toByte());

      stepTeleopManager.publishPlanningRequest(planningRequestPacket);

      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), 1.5, 1.5 * FootstepNode.gridSizeXY));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), 0.5, 1.5 * FootstepNode.gridSizeXY));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), -Math.PI * 0.25, 1.5 * FootstepNode.gridSizeYaw));
      conductor.addDurationGoal(variables.getYoTime(), 25.0);
      conductor.simulate();
   }

}
