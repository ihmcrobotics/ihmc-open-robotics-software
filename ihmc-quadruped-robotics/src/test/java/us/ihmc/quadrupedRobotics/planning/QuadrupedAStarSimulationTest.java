package us.ihmc.quadrupedRobotics.planning;

import controller_msgs.msg.dds.QuadrupedBodyOrientationMessage;
import controller_msgs.msg.dds.QuadrupedFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.QuadrupedFootstepPlanningToolboxOutputStatus;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedMultiRobotTestInterface;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;

public abstract class QuadrupedAStarSimulationTest implements QuadrupedMultiRobotTestInterface
{
   protected GoalOrientedTestConductor conductor;
   protected QuadrupedForceTestYoVariables variables;
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
      testEnvironment(DataSetName._20190327_163532_QuadrupedEnvironment0);
   }

   @Test
   public void testWalkingOverEnvironment1() throws IOException
   {
      testEnvironment(DataSetName._20190327_174535_QuadrupedEnvironment1);
   }

   @Test
   public void testWalkingOverEnvironment2() throws IOException
   {
      testEnvironment(DataSetName._20190327_175120_QuadrupedEnvironment2);
   }

   @Test
   public void testWalkingOverEnvironment3() throws IOException
   {
      testEnvironment(DataSetName._20190327_175227_QuadrupedEnvironment3);
   }

   public void testEnvironment(DataSetName dataSetName) throws IOException
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
      quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
      quadrupedTestFactory.setUseNetworking(true);
      conductor = quadrupedTestFactory.createTestConductor();
      variables = new QuadrupedForceTestYoVariables(conductor.getScs());
      stepTeleopManager = quadrupedTestFactory.getRemoteStepTeleopManager();
      stepTeleopManager.setShiftPlanBasedOnStepAdjustment(false);

      conductor.getScs().setCameraTracking(true, true, true, false);

      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      // forward footstep plan from planner to controller
      ROS2Tools.createCallbackSubscription(stepTeleopManager.getRos2Node(), QuadrupedFootstepPlanningToolboxOutputStatus.class,
                                           ROS2Tools.getTopicNameGenerator(stepTeleopManager.getRobotName(), ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX,
                                                                           ROS2TopicQualifier.OUTPUT),
                                           s -> stepTeleopManager.publishTimedStepListToController(s.takeNextData().getFootstepDataList()));

      // make a body orientation trajectory to match the step plan
      ROS2Tools.createCallbackSubscription(stepTeleopManager.getRos2Node(), QuadrupedBodyOrientationMessage.class,
                                           ROS2Tools.getTopicNameGenerator(stepTeleopManager.getRobotName(), ROS2Tools.FOOTSTEP_PLANNER_TOOLBOX,
                                                                           ROS2TopicQualifier.OUTPUT),
                                           s -> stepTeleopManager.publishBodyOrientationMessage(s.takeNextData()));

      // construct planning request
      QuadrupedFootstepPlanningRequestPacket planningRequestPacket = new QuadrupedFootstepPlanningRequestPacket();
      planningRequestPacket.getBodyPositionInWorld().set(variables.getRobotBodyX().getDoubleValue(), variables.getRobotBodyY().getDoubleValue(),
                                                         variables.getRobotBodyZ().getDoubleValue());
      planningRequestPacket.getBodyOrientationInWorld().setYawPitchRoll(variables.getBodyEstimateYaw(), variables.getBodyEstimatePitch(),
                                                                        variables.getBodyEstimateRoll());

      planningRequestPacket.getGoalPositionInWorld().set(plannerInput.getGoalPosition());
      planningRequestPacket.getGoalOrientationInWorld().setToYawQuaternion(plannerInput.getGoalYaw());
      planningRequestPacket.setRequestedFootstepPlannerType(FootstepPlannerType.A_STAR.toByte());
      planningRequestPacket.planar_regions_list_message_.set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList));

      stepTeleopManager.publishPlanningRequest(planningRequestPacket);

      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyX(), plannerInput.getQuadrupedStartPosition().getX(), 0.05));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyY(), plannerInput.getQuadrupedStartPosition().getY(), 0.05));
      conductor.addWaypointGoal(YoVariableTestGoal.doubleWithinEpsilon(variables.getRobotBodyYaw(), plannerInput.getQuadrupedStartYaw(), 0.25));
      conductor.addDurationGoal(variables.getYoTime(), 35.0);
      conductor.simulate();

      conductor.concludeTesting();
   }
}
