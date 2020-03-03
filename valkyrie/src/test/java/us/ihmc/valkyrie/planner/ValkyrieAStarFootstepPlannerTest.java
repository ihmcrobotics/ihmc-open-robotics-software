package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningStatus;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.LatticeNode;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.parameters.ValkyrieAdaptiveSwingParameters;

import java.util.function.Consumer;

import static us.ihmc.robotics.Assert.assertTrue;

public class ValkyrieAStarFootstepPlannerTest
{
   // visualize is blocking on each test
   private static final boolean visualize = !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();

   @Test
   public void testEODCinders()
   {
      testDataSet(DataSetName._20190220_172417_EOD_Cinders, 4.0, null);
   }

   @Test
   public void testSteppingStones()
   {
      testDataSet(DataSetName._20190327_163532_QuadrupedEnvironment0, 4.0, p -> {p.setMinimumFootholdPercent(0.8); p.setAstarHeuristicsWeight(5.0);});
   }

   @Test
   public void testStepAfterPitchDown()
   {
      testDataSet(DataSetName._20190219_182005_StepAfterPitchDown, 4.0, null);
   }

   @Test
   public void testRandomField()
   {
      testDataSet(DataSetName._20190219_182005_Random, 12.0, p -> {p.setIdealFootstepLength(0.45); p.setAstarHeuristicsWeight(5.0);});
   }

   @Test
   public void testLargeCinderBlockField()
   {
      testDataSet(DataSetName._20171215_214730_CinderBlockField, 12.0, p -> {p.setFlatGroundLowerThreshold(-0.2); p.setMaximumStepZ(0.2);});
   }

   @Test
   public void testSimpleGaps()
   {
      testDataSet(DataSetName._20190219_182005_SimpleGaps, 6.0, null);
   }

   private void testDataSet(DataSetName dataSetName, double timeout, Consumer<ValkyrieAStarFootstepPlannerParameters> parameterMutator)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      ValkyrieAStarFootstepPlanner planner = new ValkyrieAStarFootstepPlanner(robotModel);

      if(parameterMutator != null)
         parameterMutator.accept(planner.getParameters());

      if (visualize)
      {
         planner.launchVisualizer();

         // give planner some time to startup
         ThreadTools.sleep(3000);
      }

      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);

      ValkyrieFootstepPlanningRequestPacket requestPacket = createPlanningRequest(dataSet, timeout, planner.getParameters());
      ValkyrieFootstepPlanningStatus planningResult = new ValkyrieFootstepPlanningStatus();
      planner.addStatusCallback(planningResult::set);

      planner.handleRequestPacket(requestPacket);

      // Check that planner reports solution was found
      ValkyrieAStarFootstepPlanner.Status status = ValkyrieAStarFootstepPlanner.Status.fromByte(planningResult.getPlannerStatus());
      assertTrue("Planner did not find solution, status: " + status, status == ValkyrieAStarFootstepPlanner.Status.FOUND_SOLUTION);

      // Check that plan returned isn't empty
      assertTrue("Plan returned is empty", !planningResult.getFootstepDataList().getFootstepDataList().isEmpty());

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(ValkyrieAStarFootstepPlanner.createFootPolygons(robotModel));

      // Check that steps can be snapped
      for (int i = 0; i < planningResult.getFootstepDataList().getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepDataMessage = planningResult.getFootstepDataList().getFootstepDataList().get(i);
         FootstepNode node = new FootstepNode(footstepDataMessage.getLocation().getX(),
                                              footstepDataMessage.getLocation().getY(),
                                              footstepDataMessage.getOrientation().getYaw(),
                                              RobotSide.fromByte(footstepDataMessage.getRobotSide()));
         FootstepNodeSnapData snapData = snapper.snapFootstepNode(node);
         assertTrue("Solution step " + i + " can't be snapped", !snapData.getSnapTransform().containsNaN());
      }

      // Check that final steps are squared up at the goal
      double goalPositionEpsilon = 0.01 + 0.5 * LatticeNode.gridSizeXY + planner.getParameters().getMaximumXYWiggle();
      double goalYawEpsilon = 0.01 + 0.5 * LatticeNode.gridSizeYaw + planner.getParameters().getMaximumYawWiggle();
      int numberOfSteps = planningResult.getFootstepDataList().getFootstepDataList().size();
      assertFootAtGoal(requestPacket, planningResult.getFootstepDataList().getFootstepDataList().get(numberOfSteps - 1), goalPositionEpsilon, goalYawEpsilon);
      assertFootAtGoal(requestPacket, planningResult.getFootstepDataList().getFootstepDataList().get(numberOfSteps - 2), goalPositionEpsilon, goalYawEpsilon);

      if(visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   private void assertFootAtGoal(ValkyrieFootstepPlanningRequestPacket requestPacket,
                                 FootstepDataMessage footstepDataMessage,
                                 double goalPositionEpsilon,
                                 double goalYawEpsilon)
   {
      Pose3D footPose = new Pose3D(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());
      Pose3D desiredPose = RobotSide.LEFT.toByte() == footstepDataMessage.getRobotSide() ? requestPacket.getGoalLeftFootPose() : requestPacket.getGoalRightFootPose();
      assertTrue("Final step position is not at goal", footPose.getPosition().epsilonEquals(desiredPose.getPosition(), goalPositionEpsilon));
      assertTrue("Final step orientation doesn't match goal", MathTools.epsilonEquals(footPose.getYaw(), desiredPose.getYaw(), goalYawEpsilon));
   }

   public static ValkyrieFootstepPlanningRequestPacket createPlanningRequest(DataSet dataSet, double timeout, ValkyrieAStarFootstepPlannerParameters parameters)
   {
      ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();
      requestPacket.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataSet.getPlanarRegionsList()));
      requestPacket.setTimeout(timeout);
      parameters.setPacket(requestPacket.getParameters());

      RigidBodyTransform startPose = new RigidBodyTransform(new Quaternion(dataSet.getPlannerInput().getStartYaw(), 0.0, 0.0), new Vector3D(dataSet.getPlannerInput().getStartPosition()));
      startPose.appendTranslation(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0);
      requestPacket.getStartLeftFootPose().set(startPose);
      startPose.appendTranslation(0.0, - parameters.getIdealFootstepWidth(), 0.0);
      requestPacket.getStartRightFootPose().set(startPose);

      RigidBodyTransform goalPose = new RigidBodyTransform(new Quaternion(dataSet.getPlannerInput().getGoalYaw(), 0.0, 0.0), dataSet.getPlannerInput().getGoalPosition());
      goalPose.appendTranslation(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0);
      requestPacket.getGoalLeftFootPose().set(goalPose);
      goalPose.appendTranslation(0.0, - parameters.getIdealFootstepWidth(), 0.0);
      requestPacket.getGoalRightFootPose().set(goalPose);

      return requestPacket;
   }
}
