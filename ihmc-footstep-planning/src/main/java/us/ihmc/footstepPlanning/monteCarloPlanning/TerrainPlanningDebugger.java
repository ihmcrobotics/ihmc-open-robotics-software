package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;

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
   private MonteCarloFootstepPlanner planner;
   private MonteCarloFootstepPlannerRequest request;

   private Mat contactHeatMapImage;

   public TerrainPlanningDebugger(MonteCarloFootstepPlanner planner)
   {
      this.planner = planner;
   }

   public void setRequest(MonteCarloFootstepPlannerRequest request)
   {
      if (!enabled)
         return;

      this.request = request;
      this.offsetX = (int) (request.getTerrainMapData().getSensorOrigin().getX() * 50.0f);
      this.offsetY = (int) (request.getTerrainMapData().getSensorOrigin().getY() * 50.0f);
      refresh(request.getTerrainMapData());
   }

   public void refresh(TerrainMapData terrainMapData)
   {
      if (!enabled)
         return;

      this.offsetX = (int) (terrainMapData.getSensorOrigin().getX() * 50.0f);
      this.offsetY = (int) (terrainMapData.getSensorOrigin().getY() * 50.0f);

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

   public Mat getDisplayImage()
   {
      return stacked;
   }

   public void setEnabled(boolean enabled)
   {
      this.enabled = enabled;
   }
}
