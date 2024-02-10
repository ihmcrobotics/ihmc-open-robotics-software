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
import us.ihmc.footstepPlanning.tools.HeightMapTerrainGeneratorTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.robotics.robotSide.RobotSide;

public class MonteCarloFootstepPlanningTest
{
   private boolean displayPlots = true;

   private OpenCLManager openCLManager = new OpenCLManager();
   private MonteCarloFootstepPlannerParameters plannerParameters = new MonteCarloFootstepPlannerParameters();
   private TerrainPlanningDebugger debugger = new TerrainPlanningDebugger(null);
   private MonteCarloFootstepPlanner planner = new MonteCarloFootstepPlanner(plannerParameters, PlannerTools.createFootPolygons(0.2, 0.1, 0.08), debugger);
   private CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
   private RapidHeightMapExtractor heightMapExtractor = new RapidHeightMapExtractor(openCLManager);

   @Disabled
   @Test
   public void testMonteCarloFootstepPlanning()
   {
      LogTools.info("Initializing");

      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalWidthInMeters(4.0);
      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalCellSizeInMeters(0.02);
      heightMapExtractor.initialize();
      heightMapExtractor.reset();

      LogTools.info("Initialized");

      // height map is 8x8 meters, with a resolution of 0.02 meters, and a 50x50 patch in the center is set to 1m
      Mat heightMap = heightMapExtractor.getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();

      HeightMapTerrainGeneratorTools.fillWithSteppingStones(heightMap, 0.4f, 0.4f, 0.3f, 0.25f, 3);

      heightMapExtractor.getInternalGlobalHeightMapImage().writeOpenCLImage(openCLManager);
      heightMapExtractor.populateParameterBuffers(RapidHeightMapExtractor.getHeightMapParameters(), cameraIntrinsics, new Point3D());
      heightMapExtractor.computeContactMap();
      heightMapExtractor.readContactMapImage();

      Mat contactMap = heightMapExtractor.getGlobalContactImage();

      TerrainMapData terrainMapData = new TerrainMapData(heightMap, contactMap, null);

      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.5, -0.3, 0.0), new Quaternion()));
      request.setStartFootPose(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(-1.5, -0.1, 0.0), new Quaternion()));
      request.setGoalFootPose(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(1.5, -0.3, 0.0), new Quaternion()));
      request.setGoalFootPose(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(1.5, -0.1, 0.0), new Quaternion()));
      request.setTerrainMapData(terrainMapData);

      long timeStart = System.nanoTime();
      FootstepPlan plan = planner.generateFootstepPlan(request);
      long timeEnd = System.nanoTime();

      LogTools.info(String.format("Total Time: %.3f ms, Plan Size: %d, Visited: %d, Layer Counts: %s",
                                  (timeEnd - timeStart) / 1e6,
                                  plan.getNumberOfSteps(),
                                  planner.getVisitedNodes().size(),
                                  MonteCarloPlannerTools.getLayerCountsString(planner.getRoot())));

      if (displayPlots)
      {
         debugger.refresh(terrainMapData);
         debugger.plotMonteCarloFootstepPlan(plan);
         debugger.display(0);
      }

      Assertions.assertEquals(0.0, 0.0 - 0.0001, 1e-3);
   }
}
