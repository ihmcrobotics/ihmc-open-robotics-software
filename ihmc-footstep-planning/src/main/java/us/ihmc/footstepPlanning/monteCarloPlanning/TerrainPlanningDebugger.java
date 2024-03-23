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
import us.ihmc.euclid.geometry.ConvexPolygon2D;
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
import us.ihmc.perception.heightMap.TerrainMapDebugger;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;

import java.util.ArrayList;
import java.util.List;

public class TerrainPlanningDebugger
{
   private boolean enabled = true;
   public static final int scaleFactor = 8;

   private int offsetX = 0;
   private int offsetY = 0;
   private int height = 201;
   private int width = 201;

   private TerrainMapDebugger terrainMapDebugger = new TerrainMapDebugger(height, width, scaleFactor);
   private ContinuousWalkingStatusMessage statusMessage = new ContinuousWalkingStatusMessage();
   private IHMCROS2Publisher<FootstepDataListMessage> publisherForUI;
   private IHMCROS2Publisher<ContinuousWalkingStatusMessage> statusPublisher;
   private IHMCROS2Publisher<FootstepDataListMessage> monteCarloPlanPublisherForUI;
   private IHMCROS2Publisher<PoseListMessage> startAndGoalPublisherForUI;
   private IHMCROS2Publisher<PoseListMessage> monteCarloNodesPublisherForUI;
   private MonteCarloFootstepPlannerRequest request;

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
      offsetX = (int) (request.getTerrainMapData().getSensorOrigin().getX() * 50.0f);
      offsetY = (int) (request.getTerrainMapData().getSensorOrigin().getY() * 50.0f);
      terrainMapDebugger.setOffsets(offsetX, offsetY);

      refresh(request.getTerrainMapData());
   }

   public void renderInTheLoop(MonteCarloFootstepNode root,
                               MonteCarloFootstepPlannerRequest request,
                               SideDependentList<ConvexPolygon2D> footPolygons)
   {
      if (!enabled)
         return;

      FootstepPlan plan = MonteCarloPlannerTools.getFootstepPlanFromTree(root, request, footPolygons);
      refresh(request.getTerrainMapData());
      plotMonteCarloFootstepPlan(plan);
      plotTree(root);
      //display(1);
   }

   public void refresh(TerrainMapData terrainMapData)
   {
      if (!enabled)
         return;

      terrainMapDebugger.refresh(terrainMapData);
   }

   public void plotNode(MonteCarloFootstepNode node)
   {
      if (!enabled)
         return;

      plotRectangle(node.getState().getX32(), node.getState().getY32());
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

      plotFootPoses(terrainMapDebugger.getContactHeatMapColorImage(), poses, 1);
      plotFootPoses(terrainMapDebugger.getHeightMapColorImage(), poses, 1);
   }

   public void plotFootFramePoses(SideDependentList<FramePose3D> poses, int mode)
   {
      if (!enabled)
         return;

      SideDependentList<Pose3D> poses3D = new SideDependentList<>(new Pose3D(poses.get(RobotSide.LEFT)), new Pose3D(poses.get(RobotSide.RIGHT)));

      plotFootPoses(terrainMapDebugger.getContactHeatMapColorImage(), poses3D, mode);
      plotFootPoses(terrainMapDebugger.getHeightMapColorImage(), poses3D, mode);
   }

   public void plotMonteCarloFootstepPlan(FootstepPlan plan)
   {
      if (!enabled)
         return;

      plotMonteCarloFootstepPlan(terrainMapDebugger.getHeightMapColorImage(), request, plan);
      plotMonteCarloFootstepPlan(terrainMapDebugger.getContactHeatMapColorImage(), request, plan);
   }

   public void plotMonteCarloFootstepPlan(Mat image, MonteCarloFootstepPlannerRequest request, FootstepPlan plan)
   {
      plotFootPoses(image, request.getStartFootPoses(), 2);
      plotFootstepPlan(image, plan);
      plotFootPoses(image, request.getGoalFootPoses(), 3);
   }

   public void plotTree(MonteCarloFootstepNode root)
   {
      if (!enabled)
         return;

      plotNodeRecursive(root);
   }

   public void plotRectangle(float nodeX, float nodeY)
   {
      PerceptionDebugTools.plotRectangleNoScale(terrainMapDebugger.getContactHeatMapColorImage(),
                                                new Point2D((int) (nodeX - offsetX) * scaleFactor, (int) (nodeY - offsetY) * scaleFactor),
                                                1,
                                                PerceptionDebugTools.COLOR_PURPLE);
      PerceptionDebugTools.plotRectangleNoScale(terrainMapDebugger.getHeightMapColorImage(),
                                                new Point2D((int) (nodeX - offsetX) * scaleFactor, (int) (nodeY - offsetY) * scaleFactor),
                                                1,
                                                PerceptionDebugTools.COLOR_PURPLE);
   }

   public void plotNodeRecursive(MonteCarloFootstepNode node)
   {
      for (MonteCarloTreeNode child : node.getChildren())
      {
         MonteCarloFootstepNode childNode = (MonteCarloFootstepNode) child;
         PerceptionDebugTools.plotLine(terrainMapDebugger.getContactHeatMapColorImage(),
                                       new Point2D((int) (node.getState().getX() + height/2) * scaleFactor, (int) (node.getState().getY() + height / 2) * scaleFactor),
                                       new Point2D((int) (childNode.getState().getX() + height/2) * scaleFactor, (int) (childNode.getState().getY() + height / 2) * scaleFactor),
                                       PerceptionDebugTools.COLOR_PURPLE);

         plotNodeRecursive(childNode);
      }
   }

   public void plotAStarPlan(FootstepPlan plan)
   {
      if (!enabled)
         return;

      plotFootstepPlan(terrainMapDebugger.getHeightMapColorImage(), plan);
   }

   public void display(int delay)
   {
      if (!enabled)
         return;

      terrainMapDebugger.display(delay);
   }

   public void plotFootPoses(Mat image, SideDependentList<Pose3D> poses, int mode)
   {
      if (!enabled)
         return;

      for (RobotSide side : RobotSide.values)
      {
         Pose3D pose = new Pose3D(poses.get(side));
         Point2D point = new Point2D((pose.getX() * 50 - offsetX) * scaleFactor, (pose.getY() * 50 - offsetY) * scaleFactor);

         if (mode == 3)
            LogTools.debug(String.format("Goal: %d, %d", (int) point.getX(), (int) point.getY()));
         else if (mode == 2)
            LogTools.debug(String.format("Start: %d, %d", (int) point.getX(), (int) point.getY()));

         PerceptionDebugTools.plotTiltedRectangle(image, point, (float) -pose.getYaw(), 2 * scaleFactor, mode);
      }
   }

   public void plotFootstepPlan(Mat image, FootstepPlan plan)
   {
      if (plan == null)
         return;

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         Point3D pose = new Point3D(plan.getFootstep(i).getFootstepPose().getPosition().getX(),
                                    plan.getFootstep(i).getFootstepPose().getPosition().getY(),
                                    plan.getFootstep(i).getFootstepPose().getYaw());
         Point2D point = new Point2D((pose.getX() * 50 - offsetX) * scaleFactor, (pose.getY() * 50 - offsetY) * scaleFactor);
         PerceptionDebugTools.plotTiltedRectangle(image, point, -pose.getZ32(), 2 * scaleFactor, plan.getFootstep(i).getRobotSide() == RobotSide.LEFT ? -1 : 1);
      }
   }

   public void printScoreStats(MonteCarloFootstepNode root,
                               MonteCarloFootstepPlannerRequest request,
                               MonteCarloFootstepPlannerParameters parameters,
                               int iteration)
   {
      if (!enabled)
         return;

      // time now
      ArrayList<MonteCarloTreeNode> intermediatePath = new ArrayList<>();
      ArrayList<MonteCarloTreeNode> optimalPath = new ArrayList<>();
      MonteCarloPlannerTools.getOptimalPathByDepth(root, intermediatePath, optimalPath);

      double totalScore = 0;
      for (int i = 1; i < optimalPath.size(); i++)
      {
         MonteCarloFootstepNode footstepNode = (MonteCarloFootstepNode) optimalPath.get(i);
         MonteCarloFootstepNode previousNode = (MonteCarloFootstepNode) optimalPath.get(i - 1);
         double score = MonteCarloPlannerTools.scoreFootstepNode(previousNode, footstepNode, request, parameters, false);
         totalScore += score;
      }
      long nanoTime = System.nanoTime();
      LogTools.warn(String.format("[MCFP] Time: %d, Iteration: %d, Total Score: %.3f", nanoTime, iteration, totalScore));
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

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }

   public Mat getHeatMapImage()
   {
      return terrainMapDebugger.getContactHeatMapColorImage();
   }
}
