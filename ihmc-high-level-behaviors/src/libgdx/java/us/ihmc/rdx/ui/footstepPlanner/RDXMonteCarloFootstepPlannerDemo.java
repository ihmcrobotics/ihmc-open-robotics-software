package us.ihmc.rdx.ui.footstepPlanner;

import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlannerRequest;
import us.ihmc.footstepPlanning.monteCarloPlanning.TerrainPlanningDebugger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRequestFactory;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionDataTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.perception.RDXHumanoidPerceptionUI;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class RDXMonteCarloFootstepPlannerDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ScheduledExecutorService scheduledExecutor = ExecutorServiceTools.newScheduledThreadPool(4,
                                                                                                    getClass(),
                                                                                                    ExecutorServiceTools.ExceptionHandling.CANCEL_AND_REPORT);

   private final RDXStoredPropertySetTuner heightMapParametersTuner = new RDXStoredPropertySetTuner("Height Map Tuner");
   private final RDXStoredPropertySetTuner monteCarloPlannerParametersTuner = new RDXStoredPropertySetTuner("Monte-Carlo Tuner");
   private final SideDependentList<FramePose3D> goalPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> startPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final ROS2Helper ros2Helper;
   private final ROS2Node ros2Node;

   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private RDXPanel navigationPanel;

   private MonteCarloFootstepPlannerParameters monteCarloPlannerParameters;
   private FootstepPlannerParametersBasics footstepPlannerParameters;
   private MonteCarloFootstepPlanner monteCarloFootstepPlanner;
   private HumanoidPerceptionModule humanoidPerception;
   private HeightMapData latestHeightMapData;
   private TerrainMapData loadedTerrainMapData;
   private OpenCLManager openCLManager;

   private TerrainPlanningDebugger terrainPlanningDebugger;
   private FootstepPlan monteCarloFootstepPlan;

   private final ImFloat startMidX = new ImFloat(-1.5f);
   private final ImFloat startMidY = new ImFloat(0.0f);
   private final ImFloat startMidZ = new ImFloat(0.0f);
   private final ImFloat startYaw = new ImFloat(0.0f);

   private final ImFloat goalMidX = new ImFloat(1.5f);
   private final ImFloat goalMidY = new ImFloat(0.0f);
   private final ImFloat goalYaw = new ImFloat(0.0f);

   private final ImBoolean enableMonteCarloPlanner = new ImBoolean(false);
   private final ImBoolean resetMonteCarloPlanner = new ImBoolean(true);

   private int autoIncrementCounter = 0;
   private boolean initialized = false;

   public RDXMonteCarloFootstepPlannerDemo()
   {
      this.ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "height_map_simulation_ui");
      this.ros2Helper = new ROS2Helper(ros2Node);
      this.monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      this.terrainPlanningDebugger = new TerrainPlanningDebugger(ros2Node);
      this.monteCarloFootstepPlanner = new MonteCarloFootstepPlanner(monteCarloPlannerParameters,
                                                                     PlannerTools.createFootPolygons(0.2, 0.1, 0.08),
                                                                     terrainPlanningDebugger);

      navigationPanel = new RDXPanel("Dataset Navigation Panel");
      baseUI.getImGuiPanelManager().addPanel(navigationPanel);
      navigationPanel.setRenderMethod(this::renderNavigationPanel);
      scheduledExecutor.scheduleAtFixedRate(this::update, 0, 200, TimeUnit.MILLISECONDS);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            openCLManager = new OpenCLManager();
         }

         @Override
         public void render()
         {
            if (!initialized)
            {
               initializePerceptionModule();
               initializePerceptionUI();

               baseUI.getLayoutManager().reloadLayout();

               initialized = true;
            }

            humanoidPerception.publishExternalHeightMapImage(ros2Helper);
            humanoidPerceptionUI.update(loadedTerrainMapData.getHeightMap(), monteCarloFootstepPlanner.getDebugger().getHeatMapImage());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            scheduledExecutor.shutdown();
            baseUI.dispose();
            humanoidPerception.destroy();
            humanoidPerceptionUI.destroy();
            openCLManager.destroy();
         }
      });
   }

   public void update()
   {
      MonteCarloFootstepPlannerRequest monteCarloPlannerRequest = null;

      if (enableMonteCarloPlanner.get())
      {
         RobotSide startingSide = getCloserStartingSideToGoal();
         monteCarloPlannerRequest = FootstepPlannerRequestFactory.createMCFPRequest(loadedTerrainMapData,
                                                                                    latestHeightMapData,
                                                                                    startPose.get(RobotSide.LEFT),
                                                                                    startPose.get(RobotSide.RIGHT),
                                                                                    goalPose.get(RobotSide.LEFT),
                                                                                    goalPose.get(RobotSide.RIGHT),
                                                                                    startingSide,
                                                                                    0.4f);



         if (enableMonteCarloPlanner.get())
         {
            monteCarloPlannerRequest.setStartFootPose(RobotSide.LEFT, startPose.get(RobotSide.LEFT));
            monteCarloPlannerRequest.setStartFootPose(RobotSide.RIGHT, startPose.get(RobotSide.RIGHT));
            monteCarloPlannerRequest.setGoalFootPose(RobotSide.RIGHT, goalPose.get(RobotSide.RIGHT));
            monteCarloPlannerRequest.setGoalFootPose(RobotSide.LEFT, goalPose.get(RobotSide.LEFT));
            monteCarloPlannerRequest.setRequestedInitialStanceSide(startingSide);
         }

         // This overrides the above pose settings
         setStartAndGoalFootPosesWithSliders(monteCarloPlannerRequest);

         // perform planning based on the requests created above
         terrainPlanningDebugger.publishStartAndGoalForVisualization(startPose, goalPose);

         LogTools.warn("Monte-Carlo Planner Request: {}", monteCarloPlannerRequest);

         if (resetMonteCarloPlanner.get())
            monteCarloFootstepPlanner.reset(monteCarloPlannerRequest);

         FootstepPlan plan = monteCarloFootstepPlanner.generateFootstepPlan(monteCarloPlannerRequest);
         publishMonteCarloPlan(plan);
      }
   }

   public void publishMonteCarloPlan(FootstepPlan plan)
   {
      FootstepDataListMessage footstepDataListFromPlan = FootstepDataMessageConverter.createFootstepDataListFromPlan(plan, 1.0, 0.5);
      terrainPlanningDebugger.publishMonteCarloPlan(footstepDataListFromPlan);
      terrainPlanningDebugger.publishPlannedFootsteps(footstepDataListFromPlan);
   }

   public RobotSide getCloserStartingSideToGoal()
   {
      double leftDistance = startPose.get(RobotSide.LEFT).getPosition().distance(goalPose.get(RobotSide.LEFT).getPosition());
      double rightDistance = startPose.get(RobotSide.RIGHT).getPosition().distance(goalPose.get(RobotSide.LEFT).getPosition());

      if (leftDistance < rightDistance)
         return RobotSide.LEFT;
      else
         return RobotSide.RIGHT;
   }

   private void renderNavigationPanel()
   {
      ImGui.sliderFloat("Start Mid X", startMidX.getData(), -4.0f, 4.0f);
      ImGui.sliderFloat("Start Mid Y", startMidY.getData(), -4.0f, 4.0f);
      ImGui.sliderFloat("Start Mid Z", startMidZ.getData(), -3.0f, 3.0f);
      ImGui.sliderFloat("Start Yaw", startYaw.getData(), (float) -Math.PI, (float) Math.PI);
      ImGui.sliderFloat("Goal Mid X", goalMidX.getData(), -2.0f, 2.0f);
      ImGui.sliderFloat("Goal Mid Y", goalMidY.getData(), -2.0f, 2.0f);
      ImGui.sliderFloat("Goal Yaw", goalYaw.getData(), (float) -Math.PI, (float) Math.PI);
      ImGui.separator();
      ImGui.checkbox("Enable Monte Carlo Planner", enableMonteCarloPlanner);
      ImGui.checkbox("Reset Monte Carlo Planner", resetMonteCarloPlanner);
   }

   private void setStartAndGoalFootPosesWithSliders(MonteCarloFootstepPlannerRequest request)
   {
      request.setStartFootPose(RobotSide.LEFT,
                               new FramePose3D(ReferenceFrame.getWorldFrame(),
                                               new Point3D(startMidX.get(), startMidY.get() + 0.1, 0.0),
                                               new Quaternion(startYaw.get(), 0, 0)));
      request.setStartFootPose(RobotSide.RIGHT,
                               new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(startMidX.get(), startMidY.get() - 0.1, 0.0), new Quaternion()));
      request.setGoalFootPose(RobotSide.LEFT,
                              new FramePose3D(ReferenceFrame.getWorldFrame(),
                                              new Point3D(goalMidX.get(), goalMidY.get() + 0.1, 0.0),
                                              new Quaternion(goalYaw.get(), 0, 0)));
      request.setGoalFootPose(RobotSide.RIGHT,
                              new FramePose3D(ReferenceFrame.getWorldFrame(),
                                              new Point3D(goalMidX.get(), goalMidY.get() - 0.1, 0.0),
                                              new Quaternion(goalYaw.get(), 0, 0)));

      LogTools.info("Start Position: " + request.getStartFootPoses().get(RobotSide.LEFT).getPosition());
      LogTools.info("Goal Position: " + request.getGoalFootPoses().get(RobotSide.LEFT).getPosition());
   }

   private void initializePerceptionModule()
   {
      humanoidPerception = new HumanoidPerceptionModule(openCLManager);
      humanoidPerception.initializeHeightMapExtractor(null, new CameraIntrinsics());
      humanoidPerception.initializeOrthographicRapidRegionsExtractor(201, 201);

      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalWidthInMeters(4.0);
      RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalCellSizeInMeters(0.02);
      humanoidPerception.getRapidHeightMapExtractor().initialize();
      humanoidPerception.getRapidHeightMapExtractor().reset();

      loadedTerrainMapData = generateTerrainMapData();
   }

   public TerrainMapData generateTerrainMapData()
   {
      Mat heightMap = humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
      PerceptionDataTools.fillWithStairs(heightMap);

      humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().writeOpenCLImage(openCLManager);
      humanoidPerception.getRapidHeightMapExtractor().populateParameterBuffers(RapidHeightMapExtractor.getHeightMapParameters(), new CameraIntrinsics(), new Point3D());

      humanoidPerception.getRapidHeightMapExtractor().computeContactMap();
      humanoidPerception.getRapidHeightMapExtractor().readContactMapImage();

      Mat contactMap = humanoidPerception.getRapidHeightMapExtractor().getGlobalContactImage();
      TerrainMapData terrainMapData = new TerrainMapData(heightMap, contactMap, null);

      if (latestHeightMapData == null)
      {
         latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                 (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                 terrainMapData.getSensorOrigin().getX(),
                                                 terrainMapData.getSensorOrigin().getY());
      }
      PerceptionMessageTools.convertToHeightMapData(heightMap,
                                                    latestHeightMapData,
                                                    new Point3D(terrainMapData.getSensorOrigin()),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                    (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters());
      return terrainMapData;
   }

   private void initializePerceptionUI()
   {
      humanoidPerceptionUI = new RDXHumanoidPerceptionUI(humanoidPerception, ros2Helper);
      humanoidPerceptionUI.initializeHeightMapVisualizer(ros2Helper);
      humanoidPerceptionUI.initializeHeightMapUI(ros2Helper);
      humanoidPerceptionUI.initializeRapidRegionsUI(humanoidPerception.getOrthographicRegionsExtractor());
      humanoidPerceptionUI.initializeOrthographicVisualizer(ros2Node);
      baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI);
      baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI.getHeightMapVisualizer());
      baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI);

      LogTools.info("Monte-Carlo Parameters Save File " + monteCarloPlannerParameters.findSaveFileDirectory().getFileName().toString());
      monteCarloPlannerParametersTuner.create(monteCarloPlannerParameters, false);
      humanoidPerceptionUI.addChild(monteCarloPlannerParametersTuner);

      HeightMapParameters heightMapParameters = RapidHeightMapExtractor.getHeightMapParameters();
      LogTools.info("Height Map Parameters Save File " + heightMapParameters.findSaveFileDirectory().getFileName().toString());
      heightMapParametersTuner.create(heightMapParameters, false);
      humanoidPerceptionUI.addChild(heightMapParametersTuner);
   }

   public static void main(String[] args)
   {
      new RDXMonteCarloFootstepPlannerDemo();
   }
}
