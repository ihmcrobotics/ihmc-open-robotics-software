package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class MonteCarloFootstepPlanningDebugger
{
   private int height = 201;
   private int width = 201;

   private final Mat stacked = new Mat(height, width * 2, opencv_core.CV_8UC3);
   private final Mat heightMapColorImage = new Mat(height, width, opencv_core.CV_8UC3);
   private final Mat contactHeatMapColorImage = new Mat(height, width, opencv_core.CV_8UC3);

   Mat left = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0, 0, heightMapColorImage.cols(), heightMapColorImage.rows()));
   Mat right = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(heightMapColorImage.cols(), 0, heightMapColorImage.cols(), heightMapColorImage.rows()));

   private HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();
   private MonteCarloFootstepPlanner planner;
   private MonteCarloFootstepPlannerRequest request;

   private Mat contactHeatMapImage;

   public MonteCarloFootstepPlanningDebugger(MonteCarloFootstepPlanner planner)
   {
      this.planner = planner;
   }

   public void setRequest(MonteCarloFootstepPlannerRequest request)
   {
      this.request = request;
      refresh();
   }

   public void refresh()
   {
      this.contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(request.getContactMap().clone());
      PerceptionDebugTools.convertDepthCopyToColor(request.getHeightMap().clone(), heightMapColorImage);
      opencv_imgproc.cvtColor(contactHeatMapImage, contactHeatMapColorImage, opencv_imgproc.COLOR_BGRA2BGR);
   }

   public void plotNode(MonteCarloFootstepNode node)
   {
      PerceptionDebugTools.plotRectangleNoScale(contactHeatMapColorImage,
                                                new Point2D((int) node.getState().getX(), (int) node.getState().getY()),
                                                1,
                                                PerceptionDebugTools.COLOR_PURPLE);
      PerceptionDebugTools.plotRectangleNoScale(heightMapColorImage,
                                                new Point2D((int) node.getState().getX(), (int) node.getState().getY()),
                                                1,
                                                PerceptionDebugTools.COLOR_PURPLE);
   }

   public void plotFootstepPlan(FootstepPlan plan)
   {
      plotFootPoses(contactHeatMapColorImage, request.getStartFootPoses(), 2);
      plotFootPoses(heightMapColorImage, request.getStartFootPoses(), 2);

      plotFootsteps(contactHeatMapColorImage, plan);
      plotFootsteps(heightMapColorImage, plan);

      plotFootPoses(contactHeatMapColorImage, request.getGoalFootPoses(), 3);
      plotFootPoses(heightMapColorImage, request.getGoalFootPoses(), 3);
   }

   public void display(int delay)
   {
      LogTools.debug(String.format("Dimensions: HeightMap(%d x %d), ContactMap(%d x %d)",
                                   request.getHeightMap().rows(),
                                   request.getHeightMap().cols(),
                                   request.getContactMap().rows(),
                                   request.getContactMap().cols()));

      //MonteCarloPlannerTools.plotFootstepNodeList(planner.getVisitedNodes(), contactHeatMapColorImage);

      heightMapColorImage.copyTo(left);
      contactHeatMapColorImage.copyTo(right);

      PerceptionDebugTools.display("Display", stacked, delay, 1000);
   }

   private void plotFootPoses(Mat image, SideDependentList<Pose3D> poses, int mode)
   {
      for (RobotSide side : RobotSide.values)
      {
         Pose3D pose = new Pose3D(poses.get(side));
         PerceptionDebugTools.plotTiltedRectangle(image, new Point2D(pose.getX(), pose.getY()), (float) pose.getYaw(), 2, mode);
      }
   }

   private void plotFootsteps(Mat image, FootstepPlan plan)
   {
      if (plan == null)
         return;

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         Point3D pose = new Point3D(plan.getFootstep(i).getFootstepPose().getPosition());
         PerceptionDebugTools.plotTiltedRectangle(image,
                                                  new Point2D(pose.getX(), pose.getY()),
                                                  pose.getZ32(),
                                                  2,
                                                  plan.getFootstep(i).getRobotSide() == RobotSide.LEFT ? -1 : 1);
      }
   }

   public void printContactMap()
   {
      PerceptionDebugTools.printMat("Contact Map", request.getContactMap(), 4);
   }

   public void printHeightMap()
   {
      PerceptionDebugTools.printMat("Height Map", request.getHeightMap(), 4);
   }
}
