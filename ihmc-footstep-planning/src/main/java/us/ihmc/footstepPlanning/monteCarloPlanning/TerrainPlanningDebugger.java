package us.ihmc.footstepPlanning.monteCarloPlanning;

import behavior_msgs.msg.dds.ContinuousWalkingStatusMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;

public class TerrainPlanningDebugger
{
   private boolean enabled = true;

   private int offsetX = 0;
   private int offsetY = 0;
   private int height = 201;
   private int width = 201;
   private int scale = 7;
   private int scaledHeight = scale * height;
   private int scaledWidth = scale * width;

   private final Mat stacked = new Mat(scaledHeight * 2, scaledWidth, opencv_core.CV_8UC3);
   private final Mat heightMapColorImage = new Mat(scaledHeight, scaledWidth, opencv_core.CV_8UC3);
   private final Mat contactHeatMapColorImage = new Mat(scaledHeight, scaledWidth, opencv_core.CV_8UC3);

   Mat top = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0, 0, heightMapColorImage.cols(), heightMapColorImage.rows()));
   Mat bottom = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0,
                                                                       heightMapColorImage.rows(),
                                                                       contactHeatMapColorImage.cols(),
                                                                       contactHeatMapColorImage.rows()));

   private HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();
   private ContinuousWalkingStatusMessage statusMessage = new ContinuousWalkingStatusMessage();

   private IHMCROS2Publisher<FootstepDataListMessage> publisherForUI;
   private IHMCROS2Publisher<ContinuousWalkingStatusMessage> statusPublisher;
   private IHMCROS2Publisher<FootstepDataListMessage> monteCarloPlanPublisherForUI;
   private IHMCROS2Publisher<PoseListMessage> startAndGoalPublisherForUI;
   private IHMCROS2Publisher<PoseListMessage> monteCarloNodesPublisherForUI;
   private MonteCarloFootstepPlannerRequest request;

   private Mat contactHeatMapImage;

   public TerrainPlanningDebugger(ROS2Node ros2Node)
   {
      if (ros2Node != null)
      {
         publisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousWalkingAPI.PLANNED_FOOTSTEPS);
         statusPublisher = ROS2Tools.createPublisher(ros2Node, ContinuousWalkingAPI.CONTINUOUS_WALKING_STATUS);
         monteCarloPlanPublisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousWalkingAPI.MONTE_CARLO_FOOTSTEP_PLAN);
         startAndGoalPublisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousWalkingAPI.START_AND_GOAL_FOOTSTEPS);
         monteCarloNodesPublisherForUI = ROS2Tools.createPublisher(ros2Node, ContinuousWalkingAPI.MONTE_CARLO_TREE_NODES);
      }
   }

   public void setRequest(MonteCarloFootstepPlannerRequest request)
   {
      if (!enabled)
         return;

      this.request = request;
      this.offsetX = (int) (request.getTerrainMapData().getHeightMapCenter().getX() * 50.0f);
      this.offsetY = (int) (request.getTerrainMapData().getHeightMapCenter().getY() * 50.0f);
      refresh(request.getTerrainMapData());
   }

   public void refresh(TerrainMapData terrainMapData)
   {
      if (!enabled)
         return;

      this.offsetX = (int) (terrainMapData.getHeightMapCenter().getX() * 50.0f);
      this.offsetY = (int) (terrainMapData.getHeightMapCenter().getY() * 50.0f);

      PerceptionDebugTools.convertDepthCopyToColor(terrainMapData.getHeightMap().clone(), heightMapColorImage);
      this.contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(terrainMapData.getContactMap().clone());
      opencv_imgproc.cvtColor(contactHeatMapImage, contactHeatMapColorImage, opencv_imgproc.COLOR_BGRA2BGR);

      opencv_imgproc.resize(heightMapColorImage, heightMapColorImage, new Size(scaledWidth, scaledHeight));
      opencv_imgproc.resize(contactHeatMapColorImage, contactHeatMapColorImage, new Size(scaledWidth, scaledHeight));
   }

   public void plotNode(MonteCarloFootstepNode node)
   {
      if (!enabled)
         return;

      PerceptionDebugTools.plotRectangleNoScale(contactHeatMapColorImage,
                                                new Point2D((int) (node.getState().getX() - offsetX) * scale, (int) (node.getState().getY() - offsetY) * scale),
                                                1,
                                                PerceptionDebugTools.COLOR_PURPLE);
      PerceptionDebugTools.plotRectangleNoScale(heightMapColorImage,
                                                new Point2D((int) (node.getState().getX() - offsetX) * scale, (int) (node.getState().getY() - offsetY) * scale),
                                                1,
                                                PerceptionDebugTools.COLOR_PURPLE);
   }

   public void plotNodes(ArrayList<?> nodes)
   {
      if (!enabled)
         return;

      for (Object node : nodes)
      {
         plotNode((MonteCarloFootstepNode) node);
      }
   }

   public void plotFootPoses(SideDependentList<Pose3D> poses)
   {
      if (!enabled)
         return;

      plotFootPoses(contactHeatMapColorImage, poses, 1);
      plotFootPoses(heightMapColorImage, poses, 1);
   }

   public void plotFootstepPlan(FootstepPlan plan)
   {
      if (!enabled)
         return;

      plotFootPoses(contactHeatMapColorImage, request.getStartFootPoses(), 2);
      plotFootPoses(heightMapColorImage, request.getStartFootPoses(), 2);

      plotFootstepPlan(contactHeatMapColorImage, plan);
      plotFootstepPlan(heightMapColorImage, plan);

      plotFootPoses(contactHeatMapColorImage, request.getGoalFootPoses(), 3);
      plotFootPoses(heightMapColorImage, request.getGoalFootPoses(), 3);
   }

   public void display(int delay)
   {
      if (!enabled)
         return;

      heightMapColorImage.copyTo(top);
      contactHeatMapColorImage.copyTo(bottom);
      PerceptionDebugTools.display("Display", stacked, delay, 1500);
   }

   private void plotFootPoses(Mat image, SideDependentList<Pose3D> poses, int mode)
   {
      if (!enabled)
         return;

      for (RobotSide side : RobotSide.values)
      {
         Pose3D pose = new Pose3D(poses.get(side));
         Point2D point = new Point2D((pose.getX() * 50 - offsetX) * scale, (pose.getY() * 50 - offsetY) * scale);

         if (mode == 3)
            LogTools.debug(String.format("Goal: %d, %d", (int) point.getX(), (int) point.getY()));
         else if (mode == 2)
            LogTools.debug(String.format("Start: %d, %d", (int) point.getX(), (int) point.getY()));

         PerceptionDebugTools.plotTiltedRectangle(image, point, (float) -pose.getYaw(), 2 * scale, mode);
      }
   }

   private void plotFootstepPlan(Mat image, FootstepPlan plan)
   {
      if (plan == null)
         return;

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         Point3D pose = new Point3D(plan.getFootstep(i).getFootstepPose().getPosition().getX(),
                                    plan.getFootstep(i).getFootstepPose().getPosition().getY(),
                                    plan.getFootstep(i).getFootstepPose().getYaw());
         Point2D point = new Point2D((pose.getX() * 50 - offsetX) * scale, (pose.getY() * 50 - offsetY) * scale);
         PerceptionDebugTools.plotTiltedRectangle(image, point, -pose.getZ32(), 2 * scale, plan.getFootstep(i).getRobotSide() == RobotSide.LEFT ? -1 : 1);
      }
   }

   public void printScoreStats(MonteCarloFootstepNode root, MonteCarloFootstepPlannerRequest request, MonteCarloFootstepPlannerParameters parameters)
   {
      if (!enabled)
         return;

      ArrayList<MonteCarloTreeNode> optimalPath = new ArrayList<>();
      MonteCarloPlannerTools.getOptimalPath(root, optimalPath);

      for (int i = 1; i<optimalPath.size(); i++)
      {
         MonteCarloFootstepNode footstepNode = (MonteCarloFootstepNode) optimalPath.get(i);
         MonteCarloFootstepNode previousNode = (MonteCarloFootstepNode) optimalPath.get(i-1);
         double totalScore = MonteCarloPlannerTools.scoreFootstepNode(previousNode, footstepNode, request, parameters, true);
      }
   }

   public void publishStartAndGoalForVisualization(SideDependentList<FramePose3D> startPoses, SideDependentList<FramePose3D> goalPoses)
   {
      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(startPoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(startPoses.get(RobotSide.RIGHT)));
      poses.add(new Pose3D(goalPoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(goalPoses.get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      startAndGoalPublisherForUI.publish(poseListMessage);
   }

   public void publishMonteCarloNodesForVisualization(MonteCarloTreeNode root, TerrainMapData terrainMap)
   {
      ArrayList<Pose3D> poses = new ArrayList<>();

      addChildPosesToList(root, terrainMap, poses);
      ArrayList<MonteCarloTreeNode> optimalPath = new ArrayList<>();
      MonteCarloPlannerTools.getOptimalPath(root, optimalPath);
      for (MonteCarloTreeNode node : optimalPath)
      {
         addChildPosesToList(node, terrainMap, poses);
      }
      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      monteCarloNodesPublisherForUI.publish(poseListMessage);
   }

   public void addChildPosesToList(MonteCarloTreeNode node, TerrainMapData terrainMapData, ArrayList<Pose3D> poses)
   {
      for (MonteCarloTreeNode child : node.getChildren())
      {
         MonteCarloFootstepNode footstepNode = (MonteCarloFootstepNode) child;
         float x = footstepNode.getState().getX32() / 50.0f;
         float y = footstepNode.getState().getY32() / 50.0f;
         float z = terrainMapData.getHeightInWorld(x, y);

         if (poses.size() < 100)
            poses.add(new Pose3D(x, y, z, footstepNode.getValue(), footstepNode.getLevel(), footstepNode.getRobotSide() == RobotSide.LEFT ? 0 : 1));
      }
   }

   public void publishMonteCarloPlan(FootstepDataListMessage monteCarloPlanMessage)
   {
      monteCarloPlanPublisherForUI.publish(monteCarloPlanMessage);
   }

   public void publishPlannedFootsteps(FootstepDataListMessage plannedFootstepsMessage)
   {
      publisherForUI.publish(plannedFootstepsMessage);
   }

   public void printContactMap()
   {
      if (!enabled)
         return;

      PerceptionDebugTools.printMat("Contact Map", request.getTerrainMapData().getContactMap(), 4);
   }

   public void printHeightMap()
   {
      if (!enabled)
         return;

      PerceptionDebugTools.printMat("Height Map", request.getTerrainMapData().getHeightMap(), 4);
   }

   public void publishContinuousWalkingStatusMessage()
   {

   }

   public Mat getDisplayImage()
   {
      return stacked;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }
}
