package us.ihmc.footstepPlanning.monteCarloPlanning;

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
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.opencl.OpenCLManager;
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
         planner.getDebugger().display(plan, 0);

      Assertions.assertEquals(0.0, 0.0 - 0.0001, 1e-3);
   }
}
