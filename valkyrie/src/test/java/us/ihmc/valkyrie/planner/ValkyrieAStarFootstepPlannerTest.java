package us.ihmc.valkyrie.planner;

import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningRequestPacket;
import controller_msgs.msg.dds.ValkyrieFootstepPlanningResult;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.valkyrie.ValkyrieRobotModel;

import static us.ihmc.robotics.Assert.assertTrue;

public class ValkyrieAStarFootstepPlannerTest
{
   @Test
   public void testEODCinders()
   {
      testDataSet(DataSetName._20190220_172417_EOD_Cinders, 4.0);
   }

   @Test
   public void testSteppingStones()
   {
      testDataSet(DataSetName._20190327_163532_QuadrupedEnvironment0, 4.0);
   }

   @Test
   public void testStepAfterPitchDown()
   {
      testDataSet(DataSetName._20190219_182005_StepAfterPitchDown, 4.0);
   }

   @Test
   public void testRandomField()
   {
      testDataSet(DataSetName._20190219_182005_Random, 6.0);
   }

   @Test
   public void testLargeCinderBlockField()
   {
      testDataSet(DataSetName._20171215_214730_CinderBlockField, 6.0);
   }

   @Test
   public void testSimpleGaps()
   {
      testDataSet(DataSetName._20190219_182005_SimpleGaps, 6.0);
   }

   private void testDataSet(DataSetName dataSetName, double timeout)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      ValkyrieAStarFootstepPlanner planner = new ValkyrieAStarFootstepPlanner(robotModel);
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);

      ValkyrieFootstepPlanningRequestPacket requestPacket = createPlanningRequest(dataSet, timeout, planner.getParameters());
      ValkyrieFootstepPlanningResult planningResult = planner.handleRequestPacket(requestPacket);

      // Check that plan returned isn't empty
      assertTrue(!planningResult.getFootstepDataList().getFootstepDataList().isEmpty());

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
         assertTrue(!snapData.getSnapTransform().containsNaN());
      }

      // Check that final steps are squared up at the goal
      double goalEpsilon = 0.1;
      int numberOfSteps = planningResult.getFootstepDataList().getFootstepDataList().size();
      assertMidFootAtPose(planner.getParameters(), requestPacket, planningResult.getFootstepDataList().getFootstepDataList().get(numberOfSteps - 1), goalEpsilon);
      assertMidFootAtPose(planner.getParameters(), requestPacket, planningResult.getFootstepDataList().getFootstepDataList().get(numberOfSteps - 2), goalEpsilon);
   }

   private void assertMidFootAtPose(ValkyrieAStarFootstepPlannerParameters parameters,
                                    ValkyrieFootstepPlanningRequestPacket requestPacket,
                                    FootstepDataMessage footstepDataMessage,
                                    double goalEpsilon)
   {
      Pose3D midFootPose = new Pose3D(footstepDataMessage.getLocation(), footstepDataMessage.getOrientation());
      midFootPose.appendTranslation(0.0, 0.5 * RobotSide.fromByte(footstepDataMessage.getRobotSide()).negateIfLeftSide(parameters.getIdealFootstepWidth()), 0.0);
      assertTrue(midFootPose.epsilonEquals(requestPacket.getGoalPoses().get(0), goalEpsilon));
   }

   public ValkyrieFootstepPlanningRequestPacket createPlanningRequest(DataSet dataSet, double timeout, ValkyrieAStarFootstepPlannerParameters parameters)
   {
      ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();
      requestPacket.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(dataSet.getPlanarRegionsList()));
      requestPacket.setTimeout(timeout);
      parameters.setPacket(requestPacket.getParameters());

      RigidBodyTransform startPose = new RigidBodyTransform(new Quaternion(dataSet.getPlannerInput().getStartYaw(), 0.0, 0.0), new Vector3D(dataSet.getPlannerInput().getStartPosition()));
      startPose.appendTranslation(0.0, 0.5 * parameters.getIdealFootstepWidth(), 0.0);
      requestPacket.getLeftFootPose().set(startPose);
      startPose.appendTranslation(0.0, - parameters.getIdealFootstepWidth(), 0.0);
      requestPacket.getRightFootPose().set(startPose);

      RigidBodyTransform goalPose = new RigidBodyTransform(new Quaternion(dataSet.getPlannerInput().getGoalYaw(), 0.0, 0.0), dataSet.getPlannerInput().getGoalPosition());
      requestPacket.getGoalPoses().add().set(goalPose);
      return requestPacket;
   }
}
