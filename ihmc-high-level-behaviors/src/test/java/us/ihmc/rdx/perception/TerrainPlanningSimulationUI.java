package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.behaviors.activeMapping.ContinuousPlanningTools;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlanner;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlannerRequest;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.FootstepPlannerLoggingTools;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.neural.FootstepPredictor;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.tools.PerceptionLoggingTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepGraphic;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.io.File;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;

public class TerrainPlanningSimulationUI
{
   private final boolean USE_EXTERNAL_HEIGHT_MAP = false;
   private final double MAXIMUM_NUMBER_OF_ITERATIONS = 100000;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ROS2Helper ros2Helper;
   private final ROS2Node ros2Node;

   private ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private PerceptionDataLogger perceptionDataLogger;
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXStancePoseSelectionPanel stancePoseSelectionPanel;
   private RDXHighLevelDepthSensorSimulator steppingL515Simulator;
   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private HumanoidPerceptionModule humanoidPerception;
   private RDXEnvironmentBuilder environmentBuilder;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
   private RDXPanel navigationPanel;
   private FootstepPlannerLog log;
   private TerrainMapData loadedMapData;

   private FootstepPredictor footstepPredictor;
   private FootstepPlanningModule footstepPlanningModule;
   private HeightMapData latestHeightMapData;
   private final FootstepPlannerLogger footstepPlannerLogger;
   private final MonteCarloFootstepPlannerParameters monteCarloPlannerParameters;
   private final MonteCarloFootstepPlanner monteCarloFootstepPlanner;

   private final RDXStoredPropertySetTuner monteCarloPlannerParametersTuner = new RDXStoredPropertySetTuner("Planner Parameters (Monte-Carlo)");
   private final RDXStoredPropertySetTuner heightMapParametersTuner = new RDXStoredPropertySetTuner("Height Map Parameters (Active Mapper)");

   private final TypedNotification<FootstepPlan> footstepPlanToRenderNotificaiton = new TypedNotification<>();
   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final SideDependentList<FramePose3D> goalPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<FramePose3D> startPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<RDXFootstepGraphic> goalFootstepGraphics;
   private final SideDependentList<RDXFootstepGraphic> startFootstepGraphics;
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("CameraFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame cameraZUpFrame = new PoseReferenceFrame("CameraZUpFrame", cameraFrame);
   private final RigidBodyTransform sensorToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform sensorToGroundTransform = new RigidBodyTransform();
   private final RigidBodyTransform groundToWorldTransform = new RigidBodyTransform();

   private final StancePoseCalculator stancePoseCalculator = new StancePoseCalculator(0.5f, 0.5f, 0.1f);

   private final File planningLogDirectory = new File(IHMCCommonPaths.PLANNING_DIRECTORY.resolve("NadiaPlannerLogs").toString());
   private final File logsDirectory = new File(IHMCCommonPaths.LOGS_DIRECTORY.toString());

   private final FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
   private final File[] files = planningLogDirectory.listFiles(name -> name.toString().contains("FootstepPlannerLog"));
   private final Random random = new Random(System.currentTimeMillis());
   private final Pose3D cameraPose = new Pose3D();

   private final ImBoolean enableMonteCarloPlanner = new ImBoolean(false);
   private final ImFloat startMidX = new ImFloat(0.0f);
   private final ImFloat startMidY = new ImFloat(0.0f);
   private final ImFloat startMidZ = new ImFloat(0.0f);
   private final ImFloat startYaw = new ImFloat(0.0f);

   private final ImFloat goalMidX = new ImFloat(1.5f);
   private final ImFloat goalMidY = new ImFloat(0.0f);
   private final ImFloat goalYaw = new ImFloat(0.0f);

   private final ImBoolean enablePoseSliders = new ImBoolean(false);
   private final ImBoolean renderStartPoses = new ImBoolean(false);
   private final ImBoolean renderGoalPoses = new ImBoolean(false);

   private int autoIncrementCounter = 0;
   private int plansLoggedSoFar = 0;
   private int logIndex = 22;
   private boolean planLogged = false;
   private boolean initialized = false;
   private boolean sidednessBit = false;
   private boolean enableLogging = false;
   private boolean heightMapCaptured = false;

   public TerrainPlanningSimulationUI()
   {
      if (files != null)
         Arrays.sort(files);

      ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "height_map_simulation_ui");
      ros2Helper = new ROS2Helper(ros2Node);

      monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      monteCarloFootstepPlanner = new MonteCarloFootstepPlanner(monteCarloPlannerParameters, PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
      footstepPlanningModule = new FootstepPlanningModule("HeightMapFootstepPlanner");
      footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
      footstepPredictor = new FootstepPredictor();

      goalFootstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(PlannerTools.createFootContactPoints(0.2f, 0.1f, 0.08f), new Color(1.0f, 1.0f, 1.0f, 0.5f)),
                                                     new RDXFootstepGraphic(PlannerTools.createFootContactPoints(0.2f, 0.1f, 0.08f), new Color(1.0f, 1.0f, 1.0f, 0.5f)));
      startFootstepGraphics = new SideDependentList<>(new RDXFootstepGraphic(PlannerTools.createFootContactPoints(0.2f, 0.1f, 0.08f), new Color(0.0f, 0.0f, 0.0f, 1.0f)),
                                                      new RDXFootstepGraphic(PlannerTools.createFootContactPoints(0.2f, 0.1f, 0.08f), new Color(0.0f, 0.0f, 0.0f, 1.0f)));

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            openCLManager = new OpenCLManager();
            navigationPanel = new RDXPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(0.0, 0.25, 1.6);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            l515PoseGizmo.create(baseUI.getPrimary3DPanel());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(l515PoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(l515PoseGizmo, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(30.0));
            l515PoseGizmo.getTransformToParent().prependTranslation(1.0, 0.0, 0.0);

            stancePoseSelectionPanel = new RDXStancePoseSelectionPanel(stancePoseCalculator);
            baseUI.getPrimaryScene().addRenderableProvider(stancePoseSelectionPanel, RDXSceneLevel.VIRTUAL);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(stancePoseSelectionPanel::processImGui3DViewInput);
            baseUI.getImGuiPanelManager().addPanel(stancePoseSelectionPanel.getPanelName(), stancePoseSelectionPanel::renderImGuiWidgets);

            for (RobotSide robotSide : RobotSide.values)
            {
               goalFootstepGraphics.get(robotSide).create();
               startFootstepGraphics.get(robotSide).create();
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               baseUI.getPrimaryScene().addRenderableProvider(goalFootstepGraphics.get(robotSide), RDXSceneLevel.MODEL);
               baseUI.getPrimaryScene().addRenderableProvider(startFootstepGraphics.get(robotSide), RDXSceneLevel.MODEL);
            }

            baseUI.getPrimaryScene().addRenderableProvider(footstepPlanGraphic, RDXSceneLevel.MODEL);

            environmentBuilder.loadEnvironment("HarderTerrain.json");

            navigationPanel.setRenderMethod(this::renderNavigationPanel);
         }

         @Override
         public void render()
         {
            if (!initialized)
            {
               initializeDepthSimulator();
               initializeDepthImage();
               initializePerceptionModule();
               initializePerceptionUI();

               baseUI.getLayoutManager().reloadLayout();

               initialized = true;
            }

            if (enableLogging)
            {
               autoIncrementCounter++;
               updateFrameState(autoIncrementCounter);
            }

            steppingL515Simulator.render(baseUI.getPrimaryScene());

            Point3D position = new Point3D(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getTranslation());
            Quaternion orientation = new Quaternion(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getRotation());

            updateCameraFrames(position, orientation);

            if (USE_EXTERNAL_HEIGHT_MAP)
            {
               humanoidPerception.publishExternalHeightMapImage(ros2Helper);
            }
            else
            {
               humanoidPerception.updateTerrain(ros2Helper,
                                                steppingL515Simulator.getLowLevelSimulator().getMetersDepthOpenCVMat(),
                                                cameraFrame,
                                                cameraZUpFrame,
                                                true);
            }

            if (enableMonteCarloPlanner.get())
            {
               updateMonteCarloPlanner(true);
            }

            if (footstepPlanToRenderNotificaiton.poll())
            {
               FootstepPlan generatedPlan = footstepPlanToRenderNotificaiton.read();
               FootstepDataListMessage footstepsMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(generatedPlan, 2.0, 1.0);
               footstepPlanGraphic.generateMeshesAsync(footstepsMessage, "Height Map Simulation");
               footstepPlanGraphic.update();
            }

            if (enablePoseSliders.get())
            {
               l515PoseGizmo.getTransformToParent().getTranslation().setX(startMidX.get());
               l515PoseGizmo.getTransformToParent().getTranslation().setY(startMidY.get());
               l515PoseGizmo.getTransformToParent().getTranslation().setZ(startMidZ.get());
               l515PoseGizmo.update();
            }

            stancePoseSelectionPanel.update(goalPose.get(RobotSide.LEFT), loadedMapData);

            for (RobotSide side : RobotSide.values)
            {
               startFootstepGraphics.get(side).setRender(renderStartPoses.get());
               goalFootstepGraphics.get(side).setRender(renderGoalPoses.get());
            }

            //if (loadedMapData != null && !footstepPlanToRenderNotificaiton.hasValue())
            //{
            //   monteCarloFootstepPlanner.getDebugger().refresh(loadedMapData);
            //}

            if (log != null)
            {
               goalFootstepGraphics.get(RobotSide.LEFT).setPose(log.getRequestPacket().getGoalLeftFootPose());
               goalFootstepGraphics.get(RobotSide.RIGHT).setPose(log.getRequestPacket().getGoalRightFootPose());
               startFootstepGraphics.get(RobotSide.LEFT).setPose(log.getRequestPacket().getStartLeftFootPose());
               startFootstepGraphics.get(RobotSide.RIGHT).setPose(log.getRequestPacket().getStartRightFootPose());
            }

            humanoidPerceptionUI.update();
            footstepPlanGraphic.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void updateCameraFrames(Point3D position, Quaternion orientation)
         {
            cameraPose.set(position, orientation);
            cameraFrame.setPoseAndUpdate(cameraPose);

            sensorToWorldTransform.set(orientation, position);

            groundToWorldTransform.set(sensorToWorldTransform);
            groundToWorldTransform.getRotation().setYawPitchRoll(sensorToWorldTransform.getRotation().getYaw(), 0, 0);
            groundToWorldTransform.getTranslation().setZ(0);

            sensorToGroundTransform.set(sensorToWorldTransform);
            sensorToGroundTransform.getTranslation().setX(0.0f);
            sensorToGroundTransform.getTranslation().setY(0.0f);
            sensorToGroundTransform.getRotation()
                                   .set(new Quaternion(0.0f,
                                                       sensorToGroundTransform.getRotation().getPitch(),
                                                       sensorToGroundTransform.getRotation().getRoll()));
            RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
            groundToSensorTransform.invert();

            cameraZUpFrame.setPoseAndUpdate(groundToSensorTransform);
         }

         private void renderNavigationPanel()
         {
            if (ImGui.button("Start"))
            {
               if (perceptionDataLogger == null)
               {
                  perceptionDataLogger = new PerceptionDataLogger();
                  String logFileName = HDF5Tools.generateLogFileName();
                  FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

                  perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
                  perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME);
                  perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);
                  perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.SENSOR_CROPPED_HEIGHT_MAP_NAME);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.START_FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.START_FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.GOAL_FOOTSTEP_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.GOAL_FOOTSTEP_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_POSITION, 3, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, 4, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.INITIAL_FOOTSTEP_SIDE, 1, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
                  perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.FOOTSTEP_SIDE, 1, PerceptionLoggerConstants.LEGACY_BLOCK_SIZE);
               }

               enableLogging = true;
            }
            ImGui.sameLine();
            if (ImGui.button("Stop"))
            {
               enableLogging = false;
               logInternalHeightMap();

               if (perceptionDataLogger != null)
               {
                  perceptionDataLogger.closeLogFile();
                  perceptionDataLogger = null;
               }
            }
            ImGui.separator();
            if (ImGui.button("Capture Height Map"))
            {
               logInternalHeightMap();
            }
            if (ImGui.button("Reset"))
            {
               enableLogging = false;
               autoIncrementCounter = 0;
               l515PoseGizmo.getTransformToParent().getTranslation().setX(0);
               l515PoseGizmo.update();

               humanoidPerception.getRapidHeightMapExtractor().reset();
               humanoidPerceptionUI.update();
            }
            ImGui.separator();
            if (ImGui.button("Plan Footsteps"))
            {
               FootstepPlannerOutput output = planFootsteps(humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getHeightMap(),
                                                            cameraZUpFrame.getTransformToWorldFrame());
               printStatusAndUpdateMeshes(output);
            }
            ImGui.sameLine();
            if (ImGui.button("Predict Footsteps"))
            {
               ArrayList<Point3D> feetLocations = predictFootsteps(cameraZUpFrame.getTransformToWorldFrame());

               // set the heights to be the same as on height map
               for (Point3D feetLocation : feetLocations)
               {
                  feetLocation.setZ(latestHeightMapData.getHeightAt(feetLocation.getX(), feetLocation.getY()));
               }

               FootstepDataListMessage message = FootstepDataMessageConverter.convertFeetLocationsToMessage(feetLocations);
               footstepPlanGraphic.generateMeshesAsync(message, "Predicted Footsteps");
               footstepPlanGraphic.update();

               // translate the l515 pose forward by 0.1 meters
               l515PoseGizmo.getTransformToParent().prependTranslation(0.1, 0.0, 0.0);
               l515PoseGizmo.update();
            }
            ImGui.separator();
            if (ImGui.button("Load Log"))
            {
               loadFootstepPlannerLog();
            }
            ImGui.sameLine();
            ImGui.pushStyleColor(ImGuiCol.Button, 0.8f, 0.1f, 0.1f, 0.5f);
            if (ImGui.button("Load Previous"))
            {
               logIndex--;
               loadFootstepPlannerLog();
            }
            ImGui.popStyleColor();
            ImGui.sameLine();
            ImGui.pushStyleColor(ImGuiCol.Button, 0.1f, 0.8f, 0.1f, 0.5f);
            if (ImGui.button("Load Next"))
            {
               logIndex++;
               loadFootstepPlannerLog();
            }
            ImGui.popStyleColor();
            ImGui.checkbox("Render Start Poses", renderStartPoses);
            ImGui.checkbox("Render Goal Poses", renderGoalPoses);
            ImGui.checkbox("Use Slider Pose", enablePoseSliders);
            ImGui.sliderFloat("Start Mid X", startMidX.getData(), -4.0f, 4.0f);
            ImGui.sliderFloat("Start Mid Y", startMidY.getData(), -4.0f, 4.0f);
            ImGui.sliderFloat("Start Mid Z", startMidZ.getData(), -3.0f, 3.0f);
            ImGui.sliderFloat("Start Yaw", startYaw.getData(), (float) -Math.PI, (float) Math.PI);
            ImGui.sliderFloat("Goal Mid X", goalMidX.getData(), -2.0f, 2.0f);
            ImGui.sliderFloat("Goal Mid Y", goalMidY.getData(), -2.0f, 2.0f);
            ImGui.sliderFloat("Goal Yaw", goalYaw.getData(), (float) -Math.PI, (float) Math.PI);
            ImGui.separator();
            if (ImGui.button("Plan Steps"))
            {
               updateMonteCarloPlanner(false);
            }
            ImGui.sameLine();
            ImGui.pushStyleColor(ImGuiCol.Button, 0.5f, 0.5f, 0.5f, 0.5f);
            if (ImGui.button("Next Step"))
            {
               Vector3D translation = monteCarloFootstepPlanner.transitionToOptimal();
               l515PoseGizmo.getTransformToParent().prependTranslation(translation);
               l515PoseGizmo.update();
               updateCameraFrames(new Point3D(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getTranslation()),
                                  new Quaternion(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getRotation()));
            }
            ImGui.popStyleColor();
            ImGui.checkbox("Enable Monte Carlo Planning", enableMonteCarloPlanner);
         }

         public void loadFootstepPlannerLog()
         {
            if (files == null)
               throw new RuntimeException("No log files found in " + planningLogDirectory.getAbsolutePath());

            File file = files[logIndex];
            LogTools.info("Loading log: {}", file.getName());
            FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load(file);
            if (loadResult != FootstepPlannerLogLoader.LoadResult.LOADED)
            {
               return;
            }
            log = logLoader.getLog();
            loadedMapData = getTerrainMapData(log);

            LogTools.warn("Footstep Planner Request: Start: {}, Goal: {}", log.getRequestPacket().getStartLeftFootPose(), log.getRequestPacket().getGoalLeftFootPose());
         }

         public void updateMonteCarloPlanner(boolean reset)
         {
            if (!monteCarloFootstepPlanner.isPlanning())
            {
               executorService.clearTaskQueue();
               executorService.submit(() ->
                 {
                    FootstepPlan plan = null;

                    if (USE_EXTERNAL_HEIGHT_MAP && log != null)
                    {
                       MonteCarloFootstepPlannerRequest request = createMonteCarloFootstepPlannerRequest(loadedMapData, log);
                       plan = planFootstepsMonteCarlo(request, reset);
                       footstepPlanToRenderNotificaiton.set(plan);
                    }
                    else if (!USE_EXTERNAL_HEIGHT_MAP)
                    {
                       TerrainMapData terrainMap = humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData();
                       MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
                       request.setTerrainMapData(terrainMap);
                       //setStartAndGoalFootPosesWithSliders(request);
                       setStartAndGoalFootPosesFromSimulation(request, cameraZUpFrame.getTransformToWorldFrame());
                       plan = planFootstepsMonteCarlo(request, reset);
                       footstepPlanToRenderNotificaiton.set(plan);
                    }

                    if (plan != null)
                        monteCarloFootstepPlanner.getDebugger().plotFootstepPlan(plan);

                    if (stancePoseSelectionPanel.isSelectionActive())
                    {
                       monteCarloFootstepPlanner.getDebugger().plotFootPoses(stancePoseCalculator.getBestPoses());
                    }
                    monteCarloFootstepPlanner.getDebugger().display(1);
                 });
            }
         }

         public ArrayList<Point3D> predictFootsteps(RigidBodyTransform zUpToWorldTransform)
         {
            Mat heightMapImage = humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getHeightMap();
            if (latestHeightMapData == null)
            {
               latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                       (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                       cameraFrame.getX(),
                                                       cameraFrame.getY());
            }
            PerceptionMessageTools.convertToHeightMapData(heightMapImage,
                                                          latestHeightMapData,
                                                          new Point3D(zUpToWorldTransform.getTranslation()),
                                                          (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                          (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters());

            LogTools.info("Cropped Image Size: {} {}", heightMapImage.cols(), heightMapImage.rows());
            return footstepPredictor.generateFootsteps(heightMapImage,
                                                       new Point2D(zUpToWorldTransform.getTranslation()),
                                                       new Point2D(zUpToWorldTransform.getTranslation().getX() + 1.65,
                                                                   zUpToWorldTransform.getTranslation().getY()),
                                                       new Point2D(zUpToWorldTransform.getTranslation()));
         }

         public FootstepPlannerOutput planFootsteps(Mat heightMapImage, RigidBodyTransform zUpToWorldTransform)
         {
            if (latestHeightMapData == null)
            {
               latestHeightMapData = new HeightMapData((float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters(),
                                                       (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                       cameraFrame.getX(),
                                                       cameraFrame.getY());
            }
            PerceptionMessageTools.convertToHeightMapData(heightMapImage,
                                                          latestHeightMapData,
                                                          new Point3D(zUpToWorldTransform.getTranslation()),
                                                          (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalCellSizeInMeters(),
                                                          (float) RapidHeightMapExtractor.getHeightMapParameters().getGlobalWidthInMeters());

            ContinuousPlanningTools.generateSensorZUpToStraightGoalFootPoses(latestHeightMapData, zUpToWorldTransform, startPose, goalPose, null);
            sidednessBit = random.nextBoolean();

            FootstepPlannerRequest request = new FootstepPlannerRequest();
            request.setHeightMapData(latestHeightMapData);
            request.setStartFootPoses(startPose.get(RobotSide.LEFT), startPose.get(RobotSide.RIGHT));
            request.setGoalFootPoses(goalPose.get(RobotSide.LEFT), goalPose.get(RobotSide.RIGHT));
            request.setTimeout(0.5);
            request.setPlanBodyPath(false);
            request.setSnapGoalSteps(true);
            request.setPerformAStarSearch(true);
            request.setAssumeFlatGround(false);
            request.setSwingPlannerType(SwingPlannerType.NONE);
            request.setAbortIfGoalStepSnappingFails(true);
            request.setRequestedInitialStanceSide(sidednessBit ? RobotSide.LEFT : RobotSide.RIGHT); // 0 -> left, 1 -> right

            FootstepPlannerOutput plannerOutput = footstepPlanningModule.handleRequest(request);
            return plannerOutput;
         }

         private void setStartAndGoalFootPosesWithSliders(MonteCarloFootstepPlannerRequest request)
         {
            request.setStartFootPose(RobotSide.LEFT,
                                     new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(startMidX.get(), startMidY.get() + 0.1, 0.0),
                                                     new Quaternion(startYaw.get(), 0, 0)));
            request.setStartFootPose(RobotSide.RIGHT,
                                     new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     new Point3D(startMidX.get(), startMidY.get() - 0.1, 0.0),
                                                     new Quaternion()));
            request.setGoalFootPose(RobotSide.LEFT,
                                    new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                    new Point3D(goalMidX.get(), goalMidY.get() + 0.1, 0.0),
                                                    new Quaternion(goalYaw.get(), 0, 0)));
            request.setGoalFootPose(RobotSide.RIGHT,
                                    new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                    new Point3D(goalMidX.get(), goalMidY.get() - 0.1, 0.0),
                                                    new Quaternion(goalYaw.get(), 0, 0)));
         }

         private void setStartAndGoalFootPosesFromSimulation(MonteCarloFootstepPlannerRequest request, RigidBodyTransform transform)
         {
            Point3D startPosition = new Point3D(transform.getTranslation());
            Point3D goalPosition = new Point3D(transform.getTranslation());
            goalPosition.add(1.5, 0.0, 0.0);

            request.setStartFootPoses(new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                      new Point3D(startPosition.getX(), startPosition.getY() + 0.1, 0.0),
                                                      new Quaternion(0, 0, 0)),
                                      new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                      new Point3D(startPosition.getX(), startPosition.getY() - 0.1, 0.0),
                                                      new Quaternion()));
            request.setGoalFootPoses(new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                       new Point3D(goalPosition.getX(), goalPosition.getY() + 0.1, 0.0),
                                                       new Quaternion(0, 0, 0)),
                                        new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                       new Point3D(goalPosition.getX(), goalPosition.getY() - 0.1, 0.0),
                                                       new Quaternion()));
         }

         public TerrainMapData getTerrainMapData(FootstepPlannerLog footstepPlannerLog)
         {
            Mat heightMap = humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
            Mat contactMap = humanoidPerception.getRapidHeightMapExtractor().getGlobalContactImage();
            TerrainMapData terrainMap = new TerrainMapData(heightMap, contactMap, null);
            PerceptionMessageTools.unpackMessage(footstepPlannerLog.getRequestPacket().getHeightMapMessage(), terrainMap);

            //HeightMapTerrainGeneratorTools.fillWithSteppingStones(heightMap, 0.4f, 0.4f, 0.3f, 0.25f, 3);
            humanoidPerception.getRapidHeightMapExtractor().getSensorOrigin().set(terrainMap.getSensorOrigin());
            humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().writeOpenCLImage(openCLManager);
            humanoidPerception.getRapidHeightMapExtractor()
                              .populateParameterBuffers(RapidHeightMapExtractor.getHeightMapParameters(),
                                                        steppingL515Simulator.getCopyOfCameraParameters(),
                                                        new Point3D(terrainMap.getSensorOrigin()));
            humanoidPerception.getRapidHeightMapExtractor().computeContactMap();
            humanoidPerception.getRapidHeightMapExtractor().readContactMapImage();

            return terrainMap;
         }

         public MonteCarloFootstepPlannerRequest createMonteCarloFootstepPlannerRequest(TerrainMapData terrainMapData, FootstepPlannerLog footstepPlannerLog)
         {
            MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
            request.setTerrainMapData(terrainMapData);
            request.setStartFootPose(RobotSide.LEFT, footstepPlannerLog.getRequestPacket().getStartLeftFootPose());
            request.setStartFootPose(RobotSide.RIGHT, footstepPlannerLog.getRequestPacket().getStartRightFootPose());
            request.setGoalFootPose(RobotSide.LEFT, footstepPlannerLog.getRequestPacket().getGoalLeftFootPose());
            request.setGoalFootPose(RobotSide.RIGHT, footstepPlannerLog.getRequestPacket().getGoalRightFootPose());

            LogTools.debug("Start: {}, Goal: {}, Origin: {}", request.getStartFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.LEFT),
                           request.getTerrainMapData().getSensorOrigin());

            return request;
         }

         public FootstepPlan planFootstepsMonteCarlo(MonteCarloFootstepPlannerRequest request, boolean reset)
         {
            long timeStart = System.nanoTime();

            if (reset)
            {
               monteCarloFootstepPlanner.reset(request);
            }

            FootstepPlan plan = monteCarloFootstepPlanner.generateFootstepPlan(request);

            long timeEnd = System.nanoTime();
            LogTools.info(String.format("Total Time: %.3f ms, Plan Size: %d, Visited: %d, Layer Counts: %s",
                                        (timeEnd - timeStart) / 1e6,
                                        plan.getNumberOfSteps(),
                                        monteCarloFootstepPlanner.getVisitedNodes().size(),
                                        MonteCarloPlannerTools.getLayerCountsString(monteCarloFootstepPlanner.getRoot())));
            return plan;
         }

         public void setToRandomPose(RigidBodyTransform transformToPack)
         {
            transformToPack.getTranslation().set(1.2 + random.nextDouble() * 6.0, 1.0f + random.nextDouble() * 5.0, 1.5 + random.nextDouble() * 0.75);
            transformToPack.getRotation().setYawPitchRoll(random.nextDouble() * 2.0 * Math.PI, Math.toRadians(60.0f), 0.0f);
         }

         public void updateFrameState(int counter)
         {
            if (counter % 10 == 0 && counter < MAXIMUM_NUMBER_OF_ITERATIONS * 10)
            {
               if (planLogged)
               {
                  //humanoidPerception.getRapidHeightMapExtractor().reset();
                  setToRandomPose(l515PoseGizmo.getTransformToParent());
                  l515PoseGizmo.update();
                  planLogged = false;
                  return;
               }
            }

            if (counter % 10 == 0)
            {
               FootstepPlannerOutput footstepPlannerOutput = planFootsteps(humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getHeightMap(),
                                                                           cameraZUpFrame.getTransformToWorldFrame());
               printStatusAndUpdateMeshes(footstepPlannerOutput);

               if (footstepPlannerOutput != null)
               {
                  if (footstepPlannerOutput.getFootstepPlanningResult() != FootstepPlanningResult.FOUND_SOLUTION
                      && footstepPlannerOutput.getFootstepPlanningResult() != FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION)
                  {
                     LogTools.warn("Failed to find a solution. Not logging.");
                  }
                  else
                  {
                     if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() >= 4)
                     {
                        // assertively log the height map, start and goal poses, and the footstep plan (assuming plan is valid)
                        LogTools.info("Logging Plan No. {}", plansLoggedSoFar++);
                        FootstepPlannerLoggingTools.printFootstepPlan(footstepPlannerOutput);

                        FootstepPlannerLoggingTools.logFootsteps(footstepPlannerOutput,
                                                                 perceptionDataLogger,
                                                                 l515PoseGizmo.getTransformToParent(),
                                                                 startPose,
                                                                 goalPose,
                                                                 sidednessBit);

                        PerceptionLoggingTools.logHeightMap(perceptionDataLogger,
                                                            humanoidPerception.getRapidHeightMapExtractor().getTerrainMapData().getHeightMap(),
                                                            PerceptionLoggerConstants.CROPPED_HEIGHT_MAP_NAME);
                     }
                  }
               }

               planLogged = true;
            }
         }

         public void printStatusAndUpdateMeshes(FootstepPlannerOutput output)
         {
            if (output != null)
            {
               FootstepPlanningResult footstepPlanningResult = output.getFootstepPlanningResult();
               LogTools.info(String.format("Plan Result: %s, Steps: %d",
                                           footstepPlanningResult,
                                           footstepPlanningModule.getOutput().getFootstepPlan().getNumberOfSteps()));

               FootstepDataListMessage footstepsMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(output.getFootstepPlan(), 2.0, 1.0);

               footstepPlanGraphic.generateMeshesAsync(footstepsMessage, "Height Map Simulation");
               footstepPlanGraphic.update();
            }
         }

         private void initializeDepthSimulator()
         {
            steppingL515Simulator = RDXSimulatedSensorFactory.createRealsenseL515(l515PoseGizmo.getGizmoFrame(), () -> 0L);
            baseUI.getImGuiPanelManager().addPanel(steppingL515Simulator);
            steppingL515Simulator.setSensorEnabled(true);
            steppingL515Simulator.setPublishPointCloudROS2(false);
            steppingL515Simulator.setRenderPointCloudDirectly(false);
            steppingL515Simulator.setPublishDepthImageROS1(false);
            steppingL515Simulator.setDebugCoordinateFrame(false);
            steppingL515Simulator.setRenderColorVideoDirectly(false);
            steppingL515Simulator.setRenderDepthVideoDirectly(false);
            steppingL515Simulator.setPublishColorImageROS1(false);
            steppingL515Simulator.setPublishColorImageROS2(false);
            baseUI.getPrimaryScene().addRenderableProvider(steppingL515Simulator::getRenderables);
         }

         private void initializeDepthImage()
         {
            bytedecoDepthImage = new BytedecoImage(steppingL515Simulator.getLowLevelSimulator().getImageWidth(),
                                                   steppingL515Simulator.getLowLevelSimulator().getImageHeight(),
                                                   opencv_core.CV_16UC1);
            bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         }

         private void initializePerceptionModule()
         {
            humanoidPerception = new HumanoidPerceptionModule(openCLManager);
            humanoidPerception.initializeRealsenseDepthImage(steppingL515Simulator.getCopyOfCameraParameters().getHeight(),
                                                             steppingL515Simulator.getCopyOfCameraParameters().getWidth());
            humanoidPerception.initializeHeightMapExtractor(null, steppingL515Simulator.getLowLevelSimulator().getCameraIntrinsics());

            if (USE_EXTERNAL_HEIGHT_MAP)
            {
               RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalWidthInMeters(4.0);
               RapidHeightMapExtractor.getHeightMapParameters().setInternalGlobalCellSizeInMeters(0.02);
               humanoidPerception.getRapidHeightMapExtractor().initialize();
               humanoidPerception.getRapidHeightMapExtractor().reset();
            }
         }

         private void initializePerceptionUI()
         {
            humanoidPerceptionUI = new RDXHumanoidPerceptionUI(humanoidPerception, ros2Helper);
            humanoidPerceptionUI.initializeHeightMapVisualizer(ros2Helper);
            humanoidPerceptionUI.initializeHeightMapUI(ros2Helper);
            baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI);
            baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI.getHeightMapVisualizer());

            LogTools.info("Planner Parameters Save File " + monteCarloPlannerParameters.findSaveFileDirectory().getFileName().toString());
            monteCarloPlannerParametersTuner.create(monteCarloPlannerParameters, false);
            humanoidPerceptionUI.addChild(monteCarloPlannerParametersTuner);

            HeightMapParameters heightMapParameters = RapidHeightMapExtractor.getHeightMapParameters();
            LogTools.info("Height Map Parameters Save File " + heightMapParameters.findSaveFileDirectory().getFileName().toString());
            heightMapParametersTuner.create(heightMapParameters, false);
            humanoidPerceptionUI.addChild(heightMapParametersTuner);
         }

         public void logInternalHeightMap()
         {
            if (!heightMapCaptured)
            {
               Mat internalHeightMapImage = humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
               PerceptionLoggingTools.logHeightMap(perceptionDataLogger, internalHeightMapImage, PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME);
               heightMapCaptured = true;
            }
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            footstepPlanGraphic.destroy();
            goalFootstepGraphics.get(RobotSide.LEFT).destroy();
            goalFootstepGraphics.get(RobotSide.RIGHT).destroy();
            startFootstepGraphics.get(RobotSide.LEFT).destroy();
            startFootstepGraphics.get(RobotSide.RIGHT).destroy();
            environmentBuilder.destroy();
            steppingL515Simulator.dispose();
            humanoidPerception.destroy();
            humanoidPerceptionUI.destroy();
            stancePoseSelectionPanel.destroy();
            bytedecoDepthImage.destroy(openCLManager);
            openCLManager.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new TerrainPlanningSimulationUI();
   }
}
