package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.HeatMapGenerator;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.robotSide.RobotSide;

public class MonteCarloFootstepPlanningTest
{
   private boolean displayPlots = true;

   private OpenCLManager openCLManager = new OpenCLManager();
   private MonteCarloFootstepPlannerParameters plannerParameters = new MonteCarloFootstepPlannerParameters();
   private MonteCarloFootstepPlanner planner = new MonteCarloFootstepPlanner(plannerParameters);
   private CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
   private RapidHeightMapExtractor heightMapExtractor = new RapidHeightMapExtractor(openCLManager);

   @Disabled
   @Test
   public void testMonteCarloFootstepPlanning()
   {
      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalWidthInMeters(4.0);
      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalCellSizeInMeters(0.02);
      heightMapExtractor.initialize();
      heightMapExtractor.reset();

      // height map is 8x8 meters, with a resolution of 0.02 meters, and a 50x50 patch in the center is set to 1m
      Mat heightMap = heightMapExtractor.getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();

      // set a rectangle of size 50x50 to 1 in the center
      for (int i = 75; i < 125; i++)
      {
         for (int j = 75; j < 125; j++)
         {
            heightMap.ptr(i, j).putShort((short) (32768 + 0.35 * 32768));
         }
      }

      heightMapExtractor.getInternalGlobalHeightMapImage().writeOpenCLImage(openCLManager);
      heightMapExtractor.populateParameterBuffer(RapidHeightMapExtractor.getHeightMapParameters(), cameraIntrinsics, new Point3D());
      heightMapExtractor.computeContactMap();
      heightMapExtractor.readContactMapImage();

      Mat terrainCostImage = heightMapExtractor.getCroppedTerrainCostImage();
      Mat contactMap = heightMapExtractor.getGlobalContactImage();

      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-0.5, -0.3, 0.0), new Quaternion()));
      request.setStartFootPose(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-0.5, -0.1, 0.0), new Quaternion()));
      request.setGoalFootPose(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(2.5, -0.3, 0.0), new Quaternion()));
      request.setGoalFootPose(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(2.5, -0.1, 0.0), new Quaternion()));
      request.setContactMap(contactMap);
      request.setHeightMap(heightMap);

      long timeStart = System.nanoTime();
      FootstepPlan plan = planner.generateFootstepPlan(request);
      long timeEnd = System.nanoTime();

      LogTools.info("Total Time: {} ms, Plan Size: {}", (timeEnd - timeStart) / 1e6, plan.getNumberOfSteps());

      if (displayPlots)
         display(heightMap, contactMap, terrainCostImage, plan);

      Assertions.assertEquals(0.0, 0.0 - 0.0001, 1e-3);
   }

   public void display(Mat heightMap, Mat contactMapImage, Mat terrainCostImage, FootstepPlan plan)
   {
      HeatMapGenerator contactHeatMapGenerator = new HeatMapGenerator();
      Mat heightMapColorImage = new Mat(201, 201, opencv_core.CV_8UC3);

      Mat contactHeatMapImage = contactHeatMapGenerator.generateHeatMap(contactMapImage);

      //PerceptionDebugTools.printMat("Terrain Cost", terrainCostImage, 4);
      //PerceptionDebugTools.printMat("Contact Map", contactMapImage, 4);

      //PerceptionDebugTools.printMat("Height Map", heightMap, 4);
      //PerceptionDebugTools.printMat("Contact Map", contactMapImage, 4);
      //PerceptionDebugTools.plotFootsteps(display, null, 2);

      LogTools.info(String.format("Dimensions: HeightMap(%d x %d), ContactMap(%d x %d)",
                                  heightMap.rows(),
                                  heightMap.cols(),
                                  contactMapImage.rows(),
                                  contactMapImage.cols()));

      // stack the contact map to the right of the height map
      Mat stacked = new Mat(heightMap.rows(), heightMap.cols() + contactMapImage.cols(), opencv_core.CV_8UC3);
      Mat left = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(0, 0, heightMap.cols(), heightMap.rows()));
      Mat right = stacked.apply(new org.bytedeco.opencv.opencv_core.Rect(heightMap.cols(), 0, contactMapImage.cols(), contactMapImage.rows()));

      PerceptionDebugTools.convertDepthCopyToColor(heightMap, heightMapColorImage);
      heightMapColorImage.copyTo(left);

      //Mat contactMapColorImage = new Mat(contactMapImage.rows(), contactMapImage.cols(), opencv_core.CV_8UC3);
      //opencv_imgproc.cvtColor(contactMapImage, contactMapColorImage, opencv_imgproc.COLOR_GRAY2RGB);
      //contactMapColorImage.copyTo(right);

      // convert contact heat map from 8UC4 to 8UC3
      Mat contactHeatMapColorImage = new Mat(contactHeatMapImage.rows(), contactHeatMapImage.cols(), opencv_core.CV_8UC3);
      opencv_imgproc.cvtColor(contactHeatMapImage, contactHeatMapColorImage, opencv_imgproc.COLOR_BGRA2BGR);

      plotFootsteps(contactHeatMapColorImage, plan);
      contactHeatMapColorImage.copyTo(right);

      PerceptionDebugTools.display("Display", stacked, 0, 1000);
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
}
