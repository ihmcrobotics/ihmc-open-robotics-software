package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class MonteCarloFootstepPlanningDebugger
{
   private int height = 201;
   private int width = 201;

   private final Mat heightMapColorImage = new Mat(height, width, opencv_core.CV_8UC3);
   private final Mat stacked = new Mat(height, width * 2, opencv_core.CV_8UC3);
   private final Mat contactHeatMapColorImage = new Mat(height, width, opencv_core.CV_8UC3);

   Mat left = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0, 0, heightMapColorImage.cols(), heightMapColorImage.rows()));
   Mat right = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(heightMapColorImage.cols(), 0, heightMapColorImage.cols(), heightMapColorImage.rows()));

   private HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();
   private MonteCarloFootstepPlanner planner;
   private MonteCarloFootstepPlannerRequest request;

   public MonteCarloFootstepPlanningDebugger(MonteCarloFootstepPlanner planner)
   {
      this.planner = planner;
   }

   public void setRequest(MonteCarloFootstepPlannerRequest request)
   {
      this.request = request;
   }

   public void display(FootstepPlan plan, int delay)
   {
      Mat contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(request.getContactMap());

      LogTools.debug(String.format("Dimensions: HeightMap(%d x %d), ContactMap(%d x %d)",
                                   request.getHeightMap().rows(),
                                   request.getHeightMap().cols(),
                                   request.getContactMap().rows(),
                                   request.getContactMap().cols()));

      PerceptionDebugTools.convertDepthCopyToColor(request.getHeightMap(), heightMapColorImage);
      heightMapColorImage.copyTo(left);

      opencv_imgproc.cvtColor(contactHeatMapImage, contactHeatMapColorImage, opencv_imgproc.COLOR_BGRA2BGR);

      //MonteCarloPlannerTools.plotFootstepNodeList(planner.getVisitedNodes(), contactHeatMapColorImage);
      plotFootsteps(contactHeatMapColorImage, plan);

      contactHeatMapColorImage.copyTo(right);

      PerceptionDebugTools.display("Display", stacked, delay, 1000);
   }

   private void plotFootsteps(Mat image, FootstepPlan plan)
   {
      if (plan == null)
         return;

      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         Point3D position = new Point3D(plan.getFootstep(i).getFootstepPose().getPosition());
         PerceptionDebugTools.plotTiltedRectangle(image, position, 1, plan.getFootstep(i).getRobotSide() == RobotSide.LEFT ? -1 : 1);
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
