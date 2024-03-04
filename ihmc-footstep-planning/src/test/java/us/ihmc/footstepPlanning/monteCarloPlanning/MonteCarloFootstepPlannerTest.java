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
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public class MonteCarloFootstepPlannerTest
{
   private boolean displayPlots = true;

   private OpenCLManager openCLManager = new OpenCLManager();
   private MonteCarloFootstepPlannerParameters plannerParameters = new MonteCarloFootstepPlannerParameters();
   private TerrainPlanningDebugger debugger = new TerrainPlanningDebugger(null);
   private MonteCarloFootstepPlanner planner = new MonteCarloFootstepPlanner(plannerParameters, PlannerTools.createFootPolygons(0.2, 0.1, 0.08), debugger);
   private CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
   private RapidHeightMapExtractor heightMapExtractor = new RapidHeightMapExtractor(openCLManager);

   public void initialize()
   {
      LogTools.info("Initializing");

      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalWidthInMeters(4.0);
      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalCellSizeInMeters(0.02);
      heightMapExtractor.initialize();
      heightMapExtractor.reset();

      LogTools.info("Initialized");
   }

   public TerrainMapData generateTerrainMapData()
   {
      heightMapExtractor.getInternalGlobalHeightMapImage().writeOpenCLImage(openCLManager);
      heightMapExtractor.populateParameterBuffers(RapidHeightMapExtractor.getHeightMapParameters(), cameraIntrinsics, new Point3D());
      heightMapExtractor.computeContactMap();
      heightMapExtractor.readContactMapImage();

      Mat contactMap = heightMapExtractor.getGlobalContactImage();
      Mat heightMap = heightMapExtractor.getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
      TerrainMapData terrainMapData = new TerrainMapData(heightMap, contactMap, null);
      return terrainMapData;
   }

   public void outputDebugInfo(long timeStart, long timeEnd, TerrainMapData terrainMapData, FootstepPlan plan)
   {
      LogTools.info(String.format("Total Time: %.3f ms, Plan Size: %d, Visited: %d",
                                  (timeEnd - timeStart) / 1e6,
                                  plan.getNumberOfSteps(),
                                  planner.getVisitedNodes().size()));

      if (displayPlots)
      {
         debugger.refresh(terrainMapData);
         debugger.plotMonteCarloFootstepPlan(plan);
         debugger.display(0);
      }
   }

   public MonteCarloFootstepPlannerRequest createPlannerRequest(TerrainMapData terrainMapData,
                                                                Point3D startLeft,
                                                                Point3D startRight,
                                                                Point3D goalLeft,
                                                                Point3D goalRight)
   {
      HeightMapData heightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                      (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                      terrainMapData.getSensorOrigin().getX(),
                                                      terrainMapData.getSensorOrigin().getY());

      PerceptionMessageTools.convertToHeightMapData(terrainMapData.getHeightMap(),
                                                    heightMapData,
                                                    new Point3D(terrainMapData.getSensorOrigin()),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters());

      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), startLeft, new Quaternion()));
      request.setStartFootPose(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), startRight, new Quaternion()));
      request.setGoalFootPose(RobotSide.LEFT, new FramePose3D(ReferenceFrame.getWorldFrame(), goalLeft, new Quaternion()));
      request.setGoalFootPose(RobotSide.RIGHT, new FramePose3D(ReferenceFrame.getWorldFrame(), goalRight, new Quaternion()));
      request.setTerrainMapData(terrainMapData);
      request.setHeightMapData(heightMapData);
      request.setTimeout(plannerParameters.getTimeoutDuration());
      return request;
   }

   @Disabled
   @Test
   public void testMonteCarloFootstepPlanning()
   {

      initialize();

      // height map is 8x8 meters, with a resolution of 0.02 meters, and a 50x50 patch in the center is set to 1m
      Mat heightMap = heightMapExtractor.getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
      HeightMapTerrainGeneratorTools.fillWithSteppingStones(heightMap, 0.4f, 0.4f, 0.3f, 0.25f, 3);
      TerrainMapData terrainMapData = generateTerrainMapData();

      MonteCarloFootstepPlannerRequest plannerRequest = createPlannerRequest(terrainMapData,
                                                                             new Point3D(-1.5, -0.1, 0.0),
                                                                             new Point3D(-1.5, -0.3, 0.0),
                                                                             new Point3D(1.5, -0.1, 0.0),
                                                                             new Point3D(1.5, -0.3, 0.0));

      long timeStart = System.nanoTime();
      FootstepPlan plan = planner.generateFootstepPlan(plannerRequest);
      long timeEnd = System.nanoTime();
      outputDebugInfo(timeStart, timeEnd, terrainMapData, plan);

      Assertions.assertTrue(plan.getNumberOfSteps() > 0);
   }
}
