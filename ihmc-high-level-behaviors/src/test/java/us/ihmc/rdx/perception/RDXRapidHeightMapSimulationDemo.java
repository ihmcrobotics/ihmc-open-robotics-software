package us.ihmc.rdx.perception;

import controller_msgs.msg.dds.FootstepDataListMessage;
import imgui.ImGui;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.headless.HumanoidPerceptionModule;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXFootstepPlanGraphic;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.IHMCCommonPaths;

import java.nio.file.Paths;

public class RDXRapidHeightMapSimulationDemo
{
   private final double MAXIMUM_DISTANCE_FROM_ORIGIN = 8.0;
   private final double SENSOR_STEP_LENGTH = 0.01;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ROS2Helper ros2Helper;
   private final ROS2Node ros2Node;

   private final RDXFootstepPlanGraphic footstepPlanGraphic = new RDXFootstepPlanGraphic(PlannerTools.createFootPolygons(0.2, 0.1, 0.08));
   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();
   private RDXPose3DGizmo l515PoseGizmo = new RDXPose3DGizmo();
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXHighLevelDepthSensorSimulator steppingL515Simulator;
   private RDXHumanoidPerceptionUI humanoidPerceptionUI;
   private HumanoidPerceptionModule humanoidPerception;
   private RDXEnvironmentBuilder environmentBuilder;
   private BytedecoImage bytedecoDepthImage;
   private OpenCLManager openCLManager;
   private RDXPanel navigationPanel;

   private FootstepPlanningModule footstepPlanningModule;
   private final FootstepPlannerLogger footstepPlannerLogger;
   private FootstepPlannerRequest request;
   private FootstepPlannerOutput plannerOutput;
   private SideDependentList<FramePose3D> goalPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private SideDependentList<FramePose3D> startPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

   private HeightMapData latestHeightMapData;
   private final PoseReferenceFrame cameraFrame = new PoseReferenceFrame("CameraFrame", ReferenceFrame.getWorldFrame());
   private final PoseReferenceFrame cameraZUpFrame = new PoseReferenceFrame("CameraZUpFrame", cameraFrame);
   private final RigidBodyTransform sensorToWorldTransform = new RigidBodyTransform();
   private final RigidBodyTransform sensorToGroundTransform = new RigidBodyTransform();
   private final RigidBodyTransform groundToWorldTransform = new RigidBodyTransform();
   private final Pose3D cameraPose = new Pose3D();

   private boolean autoIncrement = false;
   private int autoIncrementCounter = 0;
   private boolean initialized = false;

   public RDXRapidHeightMapSimulationDemo()
   {
      ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "height_map_simulation_ui");
      ros2Helper = new ROS2Helper(ros2Node);

      String logFileName = HDF5Tools.generateLogFileName();
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
      perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME);

      footstepPlanningModule = new FootstepPlanningModule("HeightMapFootstepPlanner");
      footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            navigationPanel = new RDXPanel("Dataset Navigation Panel");
            baseUI.getImGuiPanelManager().addPanel(navigationPanel);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("LookAndStepHard.json");

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(2.2, 0.0, 1.6);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo = new RDXPose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            l515PoseGizmo.create(baseUI.getPrimary3DPanel());
            l515PoseGizmo.setResizeAutomatically(false);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(l515PoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(l515PoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(l515PoseGizmo, RDXSceneLevel.VIRTUAL);
            l515PoseGizmo.getTransformToParent().appendPitchRotation(Math.toRadians(30.0));

            baseUI.getPrimaryScene().addRenderableProvider(footstepPlanGraphic, RDXSceneLevel.MODEL);

            navigationPanel.setRenderMethod(this::renderNavigationPanel);
         }

         @Override
         public void render()
         {
            if (!initialized)
            {
               openCLManager = new OpenCLManager();

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

               bytedecoDepthImage = new BytedecoImage(steppingL515Simulator.getLowLevelSimulator().getImageWidth(),
                                                      steppingL515Simulator.getLowLevelSimulator().getImageHeight(),
                                                      opencv_core.CV_16UC1);
               bytedecoDepthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

               humanoidPerception = new HumanoidPerceptionModule(openCLManager);
               humanoidPerception.initializeRealsenseDepthImage(steppingL515Simulator.getCopyOfCameraParameters().getHeight(),
                                                                steppingL515Simulator.getCopyOfCameraParameters().getWidth());
               humanoidPerception.initializeHeightMapExtractor(steppingL515Simulator.getLowLevelSimulator().getCameraIntrinsics());

               humanoidPerceptionUI = new RDXHumanoidPerceptionUI(humanoidPerception, ros2Helper);
               humanoidPerceptionUI.initializeHeightMapVisualizer(ros2Helper);
               humanoidPerceptionUI.initializeHeightMapUI(ros2Helper);

               baseUI.getImGuiPanelManager().addPanel(humanoidPerceptionUI);
               baseUI.getPrimaryScene().addRenderableProvider(humanoidPerceptionUI.getHeightMapVisualizer());

               baseUI.getLayoutManager().reloadLayout();

               initialized = true;
            }

            if (autoIncrement)
            {
               autoIncrementCounter++;
               updateFrameState(autoIncrementCounter);
            }

            steppingL515Simulator.render(baseUI.getPrimaryScene());

            Point3D position = new Point3D(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getTranslation());
            Quaternion orientation = new Quaternion(l515PoseGizmo.getGizmoFrame().getTransformToWorldFrame().getRotation());
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

            Pose3D groundPoseInSensorFrame = new Pose3D(groundToSensorTransform);
            cameraZUpFrame.setPoseAndUpdate(groundPoseInSensorFrame);

            humanoidPerception.updateTerrain(ros2Helper,
                                             steppingL515Simulator.getLowLevelSimulator().getMetersDepthOpenCVMat(),
                                             cameraFrame,
                                             cameraZUpFrame,
                                             initialized,
                                             true);

            humanoidPerceptionUI.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderNavigationPanel()
         {
            if (ImGui.button("Start"))
            {
               autoIncrement = true;
            }
            ImGui.sameLine();
            if (ImGui.button("Stop"))
            {
               autoIncrement = false;
            }
            ImGui.sameLine();
            if (ImGui.button("Reset"))
            {
               autoIncrement = false;
               autoIncrementCounter = 0;
               l515PoseGizmo.getTransformToParent().getTranslation().setX(0);
               l515PoseGizmo.update();

               humanoidPerception.getRapidHeightMapExtractor().reset();
               humanoidPerceptionUI.update();

            }
            ImGui.separator();
            if(ImGui.button("Plan Footsteps"))
            {
               planFootsteps();
            }
            ImGui.separator();
            if (ImGui.button("Capture Height Map"))
            {
               logHeightMap();
            }
         }

         public void planFootsteps()
         {
            // set start pose to be 0.65m in front of the camera
            startPose.get(RobotSide.LEFT).set(cameraZUpFrame.getTransformToWorldFrame());
            startPose.get(RobotSide.LEFT).appendTranslation(0.65, 0.0, 0.0);

            startPose.get(RobotSide.RIGHT).set(cameraZUpFrame.getTransformToWorldFrame());
            startPose.get(RobotSide.RIGHT).appendTranslation(0.65, -0.2, 0.0);

            // set goal pose to be 1.65m in front of the camera
            goalPose.get(RobotSide.LEFT).set(cameraZUpFrame.getTransformToWorldFrame());
            goalPose.get(RobotSide.LEFT).appendTranslation(1.65, 0.0, 0.0);

            goalPose.get(RobotSide.RIGHT).set(cameraZUpFrame.getTransformToWorldFrame());
            goalPose.get(RobotSide.RIGHT).appendTranslation(1.65, -0.2, 0.0);

            LogTools.info("Start Poses: {} {}", startPose.get(RobotSide.LEFT).getPosition(), startPose.get(RobotSide.RIGHT).getPosition());
            LogTools.info("Goal Poses: {} {}", goalPose.get(RobotSide.LEFT).getPosition(), goalPose.get(RobotSide.RIGHT).getPosition());

            Mat heightMapImage = humanoidPerception.getRapidHeightMapExtractor().getCroppedGlobalHeightMapImage();
            if (latestHeightMapData == null)
            {
               latestHeightMapData = new HeightMapData(RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS,
                                                       RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS, cameraFrame.getX(), cameraFrame.getY());
            }
            PerceptionMessageTools.convertToHeightMapData(heightMapImage.ptr(0),
                                                          latestHeightMapData,
                                                          new Point3D(cameraFrame.getPosition()),
                                                          RapidHeightMapExtractor.GLOBAL_WIDTH_IN_METERS,
                                                          RapidHeightMapExtractor.GLOBAL_CELL_SIZE_IN_METERS);

            LogTools.info("Planning Footsteps on Height Map: {}", latestHeightMapData.getNumberOfOccupiedCells());
            request = new FootstepPlannerRequest();
            request.setHeightMapData(latestHeightMapData);
            request.setStartFootPoses(startPose.get(RobotSide.LEFT), startPose.get(RobotSide.RIGHT));
            request.setGoalFootPoses(goalPose.get(RobotSide.LEFT), goalPose.get(RobotSide.RIGHT));
            request.setTimeout(0.25);
            request.setPerformAStarSearch(true);
            request.setAssumeFlatGround(false);
            request.setSwingPlannerType(SwingPlannerType.NONE);
            request.setSnapGoalSteps(true);
            request.setAbortIfGoalStepSnappingFails(true);

            plannerOutput = footstepPlanningModule.handleRequest(request);
            footstepPlannerLogger.logSession();
            FootstepPlannerLogger.deleteOldLogs();

            if (plannerOutput != null)
            {
               FootstepPlanningResult footstepPlanningResult = plannerOutput.getFootstepPlanningResult();
               LogTools.info(String.format("Plan Result: %s, Steps: %d", footstepPlanningResult,
                                           footstepPlanningModule.getOutput().getFootstepPlan().getNumberOfSteps()));

               FootstepDataListMessage footstepsMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(plannerOutput.getFootstepPlan(), 2.0, 1.0);

               footstepPlanGraphic.generateMeshesAsync(footstepsMessage, "Height Map Simulation");
               footstepPlanGraphic.update();
            }
         }

         public void updateFrameState(int counter)
         {
            if (l515PoseGizmo.getTransformToParent().getTranslation().getX() < MAXIMUM_DISTANCE_FROM_ORIGIN)
            {
               l515PoseGizmo.getTransformToParent().prependTranslation(SENSOR_STEP_LENGTH, 0, 0);
               l515PoseGizmo.update();
            }
         }

         public void logHeightMap()
         {
            Mat internalHeightMapImage = humanoidPerception.getRapidHeightMapExtractor().getInternalGlobalHeightMapImage().getBytedecoOpenCVMat();
            BytePointer compressedDepthPointer = new BytePointer(); // deallocate later
            OpenCVTools.compressImagePNG(internalHeightMapImage, compressedDepthPointer);
            perceptionDataLogger.storeBytesFromPointer(PerceptionLoggerConstants.INTERNAL_HEIGHT_MAP_NAME, compressedDepthPointer);
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            footstepPlanGraphic.destroy();
            environmentBuilder.destroy();
            steppingL515Simulator.dispose();
            humanoidPerception.destroy();
            humanoidPerceptionUI.destroy();
            perceptionDataLogger.closeLogFile();
            bytedecoDepthImage.destroy(openCLManager);
            openCLManager.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXRapidHeightMapSimulationDemo();
   }
}
