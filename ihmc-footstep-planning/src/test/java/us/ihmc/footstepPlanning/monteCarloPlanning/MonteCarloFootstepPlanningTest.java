package us.ihmc.footstepPlanning.monteCarloPlanning;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public class MonteCarloFootstepPlanningTest
{
   @Disabled
   @Test
   public void testMonteCarloFootstepPlanning()
   {
      OpenCLManager openCLManager = new OpenCLManager();
      RapidHeightMapExtractor heightMapExtractor = new RapidHeightMapExtractor(openCLManager);
      MonteCarloFootstepPlanner planner = new MonteCarloFootstepPlanner();

      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      HeightMapParameters heightMapParameters = new HeightMapParameters("GPU");
      MonteCarloFootstepPlannerParameters plannerParameters = new MonteCarloFootstepPlannerParameters();

      // height map is 8x8 meters, with a resolution of 0.02 meters, and a 50x50 patch in the center is set to 1m
      Mat heightMap = new Mat(201, 201, opencv_core.CV_16UC1, new Scalar(32768));
      Mat heightMapColorImage = new Mat(201, 201, opencv_core.CV_8UC3);

      // set a rectangle of size 50x50 to 1 in the center
      for (int i = 75; i < 125; i++)
      {
         for (int j = 75; j < 125; j++)
         {
            heightMap.ptr(i, j).putShort((short) (32768 + 0.35 * 32768));
         }
      }

      heightMapExtractor.setGlobalHeightMapImage(heightMap);
      heightMapExtractor.populateParameterBuffer(heightMapParameters, cameraIntrinsics, new Point3D());
      heightMapExtractor.computeContactMap();
      heightMapExtractor.readContactMapImage();
      Mat contactMapImage = heightMapExtractor.getCroppedContactMapImage();

      //FootstepPlan footstepPlan = planner.generateFootstepPlan(heightMap, contactMapImage, plannerParameters);

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

      Mat contactMapColorImage = new Mat(contactMapImage.rows(), contactMapImage.cols(), opencv_core.CV_8UC3);
      opencv_imgproc.cvtColor(contactMapImage, contactMapColorImage, opencv_imgproc.COLOR_GRAY2RGB);
      contactMapColorImage.copyTo(right);

      PerceptionDebugTools.display("Display", stacked, 0, 1000);

      Assertions.assertEquals(0.0, 0.0 - 0.0001, 1e-3);
   }
}
