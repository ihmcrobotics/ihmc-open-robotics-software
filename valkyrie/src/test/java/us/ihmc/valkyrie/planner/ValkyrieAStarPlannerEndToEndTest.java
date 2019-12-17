package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner.Status;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

public class ValkyrieAStarPlannerEndToEndTest
{
   @Test
   public void testCinders()
   {
      runTest(DataSetName._20190220_172417_EOD_Cinders, 4.0, 12.0, true);
   }

   @Test
   public void testGappedPlatform()
   {
      runTest(DataSetName._20190219_182005_Hole, 4.0, 12.0, false);
   }

   private void runTest(DataSetName dataSetName, double timeout, double maxSimTime, boolean generateGroundPlane)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      new ValkyrieAStarFootstepPlanner(robotModel).setupWithRos(PubSubImplementation.FAST_RTPS);

      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);

      ValkyrieAStarFootstepPlannerParameters parameters = new ValkyrieAStarFootstepPlannerParameters();
      parameters.setMinimumFootholdPercent(0.99);
      ValkyrieFootstepPlanningRequestPacket requestPacket = ValkyrieAStarFootstepPlannerTest.createPlanningRequest(dataSet, timeout, parameters);

      Ros2Node ros2Node = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, "test_node");
      MessageTopicNameGenerator inputMessageGenerator = ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(), ValkyrieAStarFootstepPlanner.MODULE_NAME, ROS2TopicQualifier.INPUT);
      MessageTopicNameGenerator outputMessageGenerator = ROS2Tools.getTopicNameGenerator(robotModel.getSimpleRobotName(), ValkyrieAStarFootstepPlanner.MODULE_NAME, ROS2TopicQualifier.OUTPUT);

      IHMCROS2Publisher<ValkyrieFootstepPlanningRequestPacket> requestPublisher = ROS2Tools.createPublisher(ros2Node,
                                                                                                            ValkyrieFootstepPlanningRequestPacket.class,
                                                                                                            inputMessageGenerator);

      AtomicReference<ValkyrieFootstepPlanningStatus> status = new AtomicReference<>();
      ROS2Tools.createCallbackSubscription(ros2Node, ValkyrieFootstepPlanningStatus.class, outputMessageGenerator, s -> status.set(s.takeNextData()));

      requestPublisher.publish(requestPacket);
      Stopwatch stopwatch = new Stopwatch();
      stopwatch.start();

      BooleanSupplier donePlanning = () -> status.get() != null && Status.fromByte(status.get().getPlannerStatus()) != Status.PLANNING;

      while(stopwatch.lapElapsed() < 1.5 * timeout && !donePlanning.getAsBoolean())
      {
         ThreadTools.sleep(200);
      }

      Assertions.assertTrue(status.get() != null && Status.fromByte(status.get().getPlannerStatus()) == Status.FOUND_SOLUTION);

      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(dataSet.getPlanarRegionsList(), 0.02, generateGroundPlane);
      DRCSimulationTestHelper simulationTestHelper = new DRCSimulationTestHelper(new SimulationTestingParameters(), robotModel, simEnvironment);
      simulationTestHelper.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(dataSet.getPlannerInput().getStartPosition(), dataSet.getPlannerInput().getStartYaw()));
      simulationTestHelper.createSimulation(getClass().getSimpleName());

      BooleanSupplier isAtGoal = () ->
      {
         Vector3D position = new Vector3D();
         FramePose3D goalMidFoot = new FramePose3D();
         goalMidFoot.interpolate(requestPacket.getGoalLeftFootPose(), requestPacket.getGoalRightFootPose(), 0.5);
         simulationTestHelper.getRobot().getRootJoint().getPosition(position);
         double yaw = simulationTestHelper.getRobot().getRootJoint().getQuaternion().getYaw();

         double xyEpsilon = 0.1;
         double yawEpsilon = 0.1;
         return MathTools.epsilonEquals(goalMidFoot.getX(), position.getX(), xyEpsilon) && MathTools.epsilonEquals(goalMidFoot.getY(), position.getY(), xyEpsilon) && MathTools.epsilonEquals(goalMidFoot.getYaw(), yaw, yawEpsilon);
      };

      try
      {
         simulationTestHelper.simulateAndBlock(1.0);
         simulationTestHelper.publishToController(status.get().getFootstepDataList());

         while(simulationTestHelper.getRobot().getTime() < maxSimTime && !isAtGoal.getAsBoolean())
         {
            simulationTestHelper.simulateAndBlock(1.0);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         Assertions.fail(e.getMessage());
      }

      Assertions.assertTrue(isAtGoal.getAsBoolean(), "Robot did not reach goal");
   }
}
