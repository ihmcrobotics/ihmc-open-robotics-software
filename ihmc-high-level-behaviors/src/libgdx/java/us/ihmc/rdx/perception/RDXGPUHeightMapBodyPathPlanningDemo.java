package us.ihmc.rdx.perception;

import imgui.ImGui;
import imgui.type.ImBoolean;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.tools.perception.AlternateHeightMapUpdater;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParameters;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.log.LogTools;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMap;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapParameters;
import us.ihmc.perception.gpuHeightMap.SimpleGPUHeightMapUpdater;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXInteractableReferenceFrame;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXBodyPathPlanGraphic;
import us.ihmc.rdx.ui.graphics.RDXHeightMapGraphicNew;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.string.StringTools;
import us.ihmc.utilities.ros.RosMainNode;

import java.nio.FloatBuffer;
import java.util.List;

public class RDXGPUHeightMapBodyPathPlanningDemo
{
   private static final boolean USE_SIMPLE_GPU_UPDATER = false;
   private final RDXBaseUI baseUI;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private RDXHighLevelDepthSensorSimulator ouster;
   private RDXInteractableReferenceFrame robotInteractableReferenceFrame;
   private RDXSelectablePose3DGizmo ousterPoseGizmo;
   private RDXEnvironmentBuilder environmentBuilder;
   private RDXHeightMapGraphicNew heightMapGraphic;
   private SimpleGPUHeightMapParameters simpleGPUHeightMapParameters;
   private SimpleGPUHeightMapUpdater simpleGPUHeightMapUpdater;
   private RosMainNode ros1Node;
   private RDXSelectablePose3DGizmo heightMapPoseGizmo;
   private final Stopwatch bodyPathPlannerStopwatch = new Stopwatch();
   private AStarBodyPathPlanner bodyPathPlanner;
   private RDXBodyPathPlanGraphic bodyPathPlanGraphic;
   private boolean planBodyPath = false;
   private RDXSelectablePose3DGizmo startPoseGizmo;
   private RDXSelectablePose3DGizmo goalPoseGizmo;
   private final FramePose3D startFramePose = new FramePose3D();
   private final FramePose3D goalFramePose = new FramePose3D();
   private final ImBoolean updateHeightMap = new ImBoolean(true);
   private HeightMapMessage heightMapMessage;
   private AlternateHeightMapUpdater heightMapUpdater;

   public RDXGPUHeightMapBodyPathPlanningDemo(RDXBaseUI baseUI, DRCRobotModel robotModel)
   {
      this.baseUI = baseUI;
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("CurvingBlockPathForBodyPath2.json");

            robotInteractableReferenceFrame = new RDXInteractableReferenceFrame();
            robotInteractableReferenceFrame.create(ReferenceFrame.getWorldFrame(), 0.15, baseUI.getPrimary3DPanel());
            robotInteractableReferenceFrame.getTransformToParent().getTranslation().add(0.0, 0.0, 1.7);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(robotInteractableReferenceFrame::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(robotInteractableReferenceFrame::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            ousterPoseGizmo = new RDXSelectablePose3DGizmo(robotInteractableReferenceFrame.getRepresentativeReferenceFrame());
            ousterPoseGizmo.create(baseUI.getPrimary3DPanel());
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(ousterPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> ousterPoseGizmo.process3DViewInput(input));
            baseUI.getPrimaryScene().addRenderableProvider(ousterPoseGizmo::getVirtualRenderables, RDXSceneLevel.VIRTUAL);
            ousterPoseGizmo.getPoseGizmo().getTransformToParent().appendPitchRotation(Math.toRadians(60.0));

            //            ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "height_map_planning_demo_ui");

            ouster = RDXSimulatedSensorFactory.createOusterLidar(ousterPoseGizmo.getPoseGizmo().getGizmoFrame(), () -> 0L);
            //                  ouster.setupForROS1PointCloud(ros1Node, RosTools.OUSTER_POINT_CLOUD);
            baseUI.getImGuiPanelManager().addPanel(ouster);
            ouster.setSensorEnabled(true);
            ouster.setRenderPointCloudDirectly(true);
            //                  ouster.setPublishPointCloudROS1(true);
            baseUI.getPrimaryScene().addRenderableProvider(ouster::getRenderables);

            heightMapPoseGizmo = new RDXSelectablePose3DGizmo();
            heightMapPoseGizmo.create(baseUI.getPrimary3DPanel());
            heightMapPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(1.7, 0.0, 0.0);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(heightMapPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> heightMapPoseGizmo.process3DViewInput(input));
            baseUI.getPrimaryScene().addRenderableProvider(heightMapPoseGizmo::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            simpleGPUHeightMapParameters = new SimpleGPUHeightMapParameters();
            RDXStoredPropertySetTuner heightMapParameterTuner = new RDXStoredPropertySetTuner("Height Map Parameters");
            heightMapParameterTuner.create(simpleGPUHeightMapParameters, () ->
            {
            });
            baseUI.getImGuiPanelManager().addPanel(heightMapParameterTuner);
            if (USE_SIMPLE_GPU_UPDATER)
            {
               simpleGPUHeightMapUpdater = new SimpleGPUHeightMapUpdater(simpleGPUHeightMapParameters);
               simpleGPUHeightMapUpdater.create(ouster.getLowLevelSimulator().getImageWidth(),
                                                ouster.getLowLevelSimulator().getImageHeight(),
                                                ouster.getLowLevelSimulator().getMetersDepthFloatBuffer(),
                                                ouster.getDepthCameraIntrinsics().getFx(),
                                                ouster.getDepthCameraIntrinsics().getFy(),
                                                ouster.getDepthCameraIntrinsics().getCx(),
                                                ouster.getDepthCameraIntrinsics().getCy());
            }
            else
            {
               heightMapUpdater = new AlternateHeightMapUpdater();
            }

            heightMapGraphic = new RDXHeightMapGraphicNew();
            baseUI.getPrimaryScene().addRenderableProvider(heightMapGraphic, RDXSceneLevel.VIRTUAL);
            baseUI.getImGuiPanelManager().addPanel("Height Map", this::renderHeightMapImGuiWidgets);

            DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
            SideDependentList<ConvexPolygon2D> footPolygons = FootstepPlanningModuleLauncher.createFootPolygons(robotModel);
            bodyPathPlanner = new AStarBodyPathPlanner(footstepPlannerParameters, new AStarBodyPathPlannerParameters(), footPolygons, bodyPathPlannerStopwatch);

            bodyPathPlanGraphic = new RDXBodyPathPlanGraphic();
            baseUI.getPrimaryScene().addRenderableProvider(bodyPathPlanGraphic, RDXSceneLevel.VIRTUAL);

            startPoseGizmo = new RDXSelectablePose3DGizmo();
            startPoseGizmo.create(baseUI.getPrimary3DPanel());
            startPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(0.0, 0.0, 0.0);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(startPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> startPoseGizmo.process3DViewInput(input));
            baseUI.getPrimaryScene().addRenderableProvider(startPoseGizmo::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            goalPoseGizmo = new RDXSelectablePose3DGizmo();
            goalPoseGizmo.create(baseUI.getPrimary3DPanel());
            goalPoseGizmo.getPoseGizmo().getTransformToParent().getTranslation().set(1.839, -1.142, 0.0);
            goalPoseGizmo.getPoseGizmo().getTransformToParent().getRotation().appendYawRotation(-1.3767390862107274);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(goalPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(input -> goalPoseGizmo.process3DViewInput(input));
            baseUI.getPrimaryScene().addRenderableProvider(goalPoseGizmo::getVirtualRenderables, RDXSceneLevel.VIRTUAL);

            //                  ros1Node.execute();
         }

         @Override
         public void render()
         {

            ouster.render(baseUI.getPrimaryScene());

            if (updateHeightMap.get())
            {
               RigidBodyTransform heightMapToWorld = heightMapPoseGizmo.getPoseGizmo().getGizmoFrame().getTransformToWorldFrame();
               RigidBodyTransform sensorTransformToWorld = ousterPoseGizmo.getPoseGizmo().getGizmoFrame().getTransformToWorldFrame();
               if (USE_SIMPLE_GPU_UPDATER)
               {
                  simpleGPUHeightMapUpdater.computeFromDepthMap((float) heightMapToWorld.getTranslationX(),
                                                                (float) heightMapToWorld.getTranslationY(),
                                                                sensorTransformToWorld);
                  SimpleGPUHeightMap heightMap = simpleGPUHeightMapUpdater.getHeightMap();
                  heightMapMessage = heightMap.buildMessage();
               }
               else
               {
                  FloatBuffer pointCloudBuffer = ouster.getLowLevelSimulator().getPointCloudBuffer();
                  Point3D[] scanPoints = new Point3D[pointCloudBuffer.limit() / 8];
                  FramePoint3D scanPoint = new FramePoint3D();
                  for (int i = 0; i < pointCloudBuffer.limit(); i += 8)
                  {
                     scanPoint.setToZero(ReferenceFrame.getWorldFrame());
                     scanPoint.set(pointCloudBuffer.get(i), pointCloudBuffer.get(i + 1), pointCloudBuffer.get(i + 2));
                     scanPoint.changeFrame(ouster.getSensorFrame());
                     scanPoints[i / 8] = new Point3D(scanPoint);
                  }

                  // Center the height map between the start and goal
                  Point3D center = new Point3D();
                  center.set(startPoseGizmo.getPoseGizmo().getPose().getPosition());
                  center.interpolate(goalPoseGizmo.getPoseGizmo().getPose().getPosition(), 0.5);

                  heightMapMessage = heightMapUpdater.update(scanPoints, ouster.getSensorFrame(), center);
               }

               heightMapGraphic.getTransformToWorld().set(new RigidBodyTransform());
               heightMapGraphic.generateMeshesAsync(heightMapMessage);
               heightMapGraphic.update();
            }

            if (planBodyPath)
            {
               planBodyPath = false;

               HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
               heightMapData.setEstimatedGroundHeight(-1.0);

               bodyPathPlanner.clearLoggedData();
               bodyPathPlanner.setHeightMapData(heightMapData);
               FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
               startFramePose.setToZero(startPoseGizmo.getPoseGizmo().getGizmoFrame());
               startFramePose.getPosition().setY(0.2);
               startFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               footstepPlannerRequest.getStartFootPoses().put(RobotSide.LEFT, new Pose3D(startFramePose));
               startFramePose.setToZero(startPoseGizmo.getPoseGizmo().getGizmoFrame());
               startFramePose.getPosition().setY(-0.2);
               startFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               footstepPlannerRequest.getStartFootPoses().put(RobotSide.RIGHT, new Pose3D(startFramePose));
               goalFramePose.setToZero(goalPoseGizmo.getPoseGizmo().getGizmoFrame());
               goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               LogTools.info(StringTools.tupleString(goalFramePose.getPosition()));
               LogTools.info("Yaw: {}", goalFramePose.getOrientation().getYaw());
               goalFramePose.setToZero(goalPoseGizmo.getPoseGizmo().getGizmoFrame());
               goalFramePose.getPosition().setY(0.2);
               goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               footstepPlannerRequest.getGoalFootPoses().put(RobotSide.LEFT, new Pose3D(goalFramePose));
               goalFramePose.setToZero(goalPoseGizmo.getPoseGizmo().getGizmoFrame());
               goalFramePose.getPosition().setY(-0.2);
               goalFramePose.changeFrame(ReferenceFrame.getWorldFrame());
               footstepPlannerRequest.getGoalFootPoses().put(RobotSide.RIGHT, new Pose3D(goalFramePose));
               footstepPlannerRequest.setTimeout(10.0);
               //                  FootstepPlannerOutput footstepPlannerOutput = new FootstepPlannerOutput();
               //                  bodyPathPlanner.handleRequest(footstepPlannerRequest, footstepPlannerOutput);
               //                  List<Pose3D> bodyPathWaypoints = footstepPlannerOutput.getBodyPath();
               //
               //                  bodyPathPlanGraphic.generateMeshes(bodyPathWaypoints);
               //                  bodyPathPlanGraphic.update();

               FootstepPlanningModule footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(robotModel);
               heightMapData.setEstimatedGroundHeight(-1.0);
               footstepPlannerRequest.setHeightMapData(heightMapData);
               footstepPlannerRequest.setPlanBodyPath(true);
               footstepPlanningModule.handleRequest(footstepPlannerRequest);
               FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanningModule);
               footstepPlannerLogger.logSession();

               List<Pose3D> bodyPathWaypoints = footstepPlanningModule.getOutput().getBodyPath();

               bodyPathPlanGraphic.generateMeshes(bodyPathWaypoints);
               bodyPathPlanGraphic.update();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderHeightMapImGuiWidgets()
         {
            ImGui.checkbox(labels.get("Update height map"), updateHeightMap);
            ImGui.checkbox(labels.get("Ouster Gizmo"), ousterPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Height Map Gizmo"), heightMapPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Start Pose Gizmo"), startPoseGizmo.getSelected());
            ImGui.checkbox(labels.get("Goal Pose Gizmo"), goalPoseGizmo.getSelected());
            if (ImGui.button(labels.get("Plan body path")))
            {
               planBodyPath = true;
            }
         }

         @Override
         public void dispose()
         {
            bodyPathPlanner.halt();
            ros1Node.shutdown();
            environmentBuilder.destroy();
            bodyPathPlanGraphic.destroy();
            ouster.dispose();
            if (USE_SIMPLE_GPU_UPDATER)
               simpleGPUHeightMapUpdater.destroy();
            heightMapGraphic.destroy();
            baseUI.dispose();
         }
      });
   }
}
