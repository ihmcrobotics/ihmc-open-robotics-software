package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.*;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerCommunicationProperties;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
import us.ihmc.quadrupedRobotics.QuadrupedTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AStarPawStepSimulationTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedTestYoVariables variables;
   private RemoteQuadrupedTeleopManager stepTeleopManager;
   private QuadrupedTestFactory quadrupedTestFactory;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      quadrupedTestFactory = createQuadrupedTestFactory();
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
   public void testWalkingOverEnvironment0() throws IOException
   {
      testEnvironment(DataSetName._20190327_163532_QuadrupedEnvironment0, true);
   }

   @Test
   public void testWalkingOverEnvironment1() throws IOException
   {
      testEnvironment(DataSetName._20190327_174535_QuadrupedEnvironment1, false);
   }

   @Test
   public void testWalkingOverEnvironment2() throws IOException
   {
      testEnvironment(DataSetName._20190327_175120_QuadrupedEnvironment2, false);
   }

   @Test
   public void testWalkingOverEnvironment3() throws IOException
   {
      testEnvironment(DataSetName._20190327_175227_QuadrupedEnvironment3, false);
   }

   @Test
   public void testWalkingOverPlatformEnvironment() throws IOException
   {
      testEnvironment(DataSetName._20190514_163532_QuadrupedPlatformEnvironment, false);
   }

   @Test
   public void testWalkingOverShortPlatformEnvironment() throws IOException
   {
      testEnvironment(DataSetName._20190514_163532_QuadrupedShortPlatformEnvironment, false);
   }


   public void testEnvironment(DataSetName dataSetName, boolean stepsAreAdjustable) throws IOException
   {
      SimulationConstructionSetParameters simulationConstructionSetParameters = SimulationConstructionSetParameters.createFromSystemProperties();
      simulationConstructionSetParameters.setUseAutoGroundGraphics(false);

      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();
      PlannerInput plannerInput = dataSet.getPlannerInput();

      TerrainObject3D simulationEnvironment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.02, false).getTerrainObject3D();

      QuadrupedInitialOffsetAndYaw offset = new QuadrupedInitialOffsetAndYaw(plannerInput.getQuadrupedStartPosition(), plannerInput.getQuadrupedStartYaw());

      quadrupedTestFactory.setScsParameters(simulationConstructionSetParameters);
      quadrupedTestFactory.setTerrainObject3D(simulationEnvironment);
      quadrupedTestFactory.setInitialOffset(offset);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);

      stepTeleopManager.getXGaitSettings().setQuadrupedSpeed(QuadrupedSpeed.FAST);

      ROS2Topic footstepPlannerOutputTopic = PawStepPlannerCommunicationProperties.outputTopic(quadrupedTestFactory.getRobotName());

      ROS2Tools.createCallbackSubscriptionTypeNamed(stepTeleopManager.getROS2Node(), PawStepPlanningToolboxOutputStatus.class, footstepPlannerOutputTopic,
                                           s -> processFootstepPlanningOutputStatus(s.takeNextData(), stepsAreAdjustable));

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      stepTeleopManager.publishXGaitSettings(stepTeleopManager.getXGaitSettings());

      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList);
      stepTeleopManager.submitPlanarRegionsList(planarRegionsList);

      // construct planning request
      PawStepPlanningRequestPacket planningRequestPacket = new PawStepPlanningRequestPacket();
      planningRequestPacket.getBodyPositionInWorld().set(variables.getRobotBodyX().getDoubleValue(), variables.getRobotBodyY().getDoubleValue(),
                                                         variables.getRobotBodyZ().getDoubleValue());
      planningRequestPacket.getBodyOrientationInWorld().setYawPitchRoll(variables.getBodyEstimateYaw(), variables.getBodyEstimatePitch(),
                                                                        variables.getBodyEstimateRoll());

      planningRequestPacket.getGoalPositionInWorld().set(plannerInput.getGoalPosition());
      planningRequestPacket.getGoalOrientationInWorld().setToYawOrientation(plannerInput.getGoalYaw());
      planningRequestPacket.setRequestedPawPlannerType(PawStepPlannerType.A_STAR.toByte());
      planningRequestPacket.getPlanarRegionsListMessage().set(planarRegionsListMessage);
      planningRequestPacket.setTimeout(plannerInput.getQuadrupedTimeout());

      stepTeleopManager.publishPlanningRequest(planningRequestPacket);

      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), plannerInput.getQuadrupedStartPosition().getX(), 0.05));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), plannerInput.getQuadrupedStartPosition().getY(), 0.05));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), plannerInput.getQuadrupedStartYaw(), 0.25));
      conductor.addDurationGoal(variables.getYoTime(), 35.0);
      conductor.simulate();

      conductor.concludeTesting();
   }

   private void processFootstepPlanningOutputStatus(PawStepPlanningToolboxOutputStatus packet, boolean stepsAreAdjustable)
   {
      QuadrupedTimedStepListMessage footstepDataListMessage = packet.getFootstepDataList();
      PawStepPlanningResult result = PawStepPlanningResult.fromByte(packet.getFootstepPlanningResult());

      assertTrue(result.validForExecution());

      PawStepPlan pawStepPlan = convertToFootstepPlan(footstepDataListMessage);

      QuadrupedTimedStepListMessage stepMessages = new QuadrupedTimedStepListMessage();
      for (int i = 0; i < pawStepPlan.getNumberOfSteps(); i++)
      {
         QuadrupedTimedStepMessage stepMessage = stepMessages.getQuadrupedStepList().add();
         QuadrupedTimedStep step = pawStepPlan.getPawStep(i);

         stepMessage.getQuadrupedStepMessage().setRobotQuadrant(step.getRobotQuadrant().toByte());
         stepMessage.getQuadrupedStepMessage().getGoalPosition().set(step.getGoalPosition());
         stepMessage.getQuadrupedStepMessage().setGroundClearance(step.getGroundClearance());
         stepMessage.getTimeInterval().setStartTime(step.getTimeInterval().getStartTime());
         stepMessage.getTimeInterval().setEndTime(step.getTimeInterval().getEndTime());
      }

      stepMessages.setIsExpressedInAbsoluteTime(false);
      stepMessages.setAreStepsAdjustable(stepsAreAdjustable);

      stepTeleopManager.publishTimedStepListToController(stepMessages);
   }

   private static PawStepPlan convertToFootstepPlan(QuadrupedTimedStepListMessage footstepDataListMessage)
   {
      PawStepPlan pawStepPlan = new PawStepPlan();

      for (QuadrupedTimedStepMessage timedStepMessage : footstepDataListMessage.getQuadrupedStepList())
      {
         QuadrupedStepMessage stepMessage = timedStepMessage.getQuadrupedStepMessage();
         TimeIntervalMessage timeInterval = timedStepMessage.getTimeInterval();
         FramePoint3D stepPosition = new FramePoint3D();
         stepPosition.set(stepMessage.getGoalPosition());
         pawStepPlan.addPawStep(RobotQuadrant.fromByte(stepMessage.getRobotQuadrant()), stepPosition, stepMessage.getGroundClearance(), timeInterval.getStartTime(), timeInterval.getEndTime());
      }

      return pawStepPlan;
   }
}
